"""
Accounts for the SeaYou ground station.

Deliberately small: standard library only, no database, no framework. That
is partly so it is easy to read and partly so PyInstaller can bundle the
whole ground station into one .exe without dragging in native extensions.

--------------------------------------------------------------------------
The security model, stated plainly
--------------------------------------------------------------------------
This gates access to a machine that flies. Two roles:

    viewer  - sees telemetry and video. Cannot move the aircraft.
    pilot   - may take control, arm, and fly.

**New accounts are viewers.** Handing someone the address and a password
should not hand them the controls; promoting them to pilot is a separate,
deliberate act. That is the whole reason roles exist here rather than a
single shared password.

What this is NOT: it is not protection against a determined attacker on a
hostile network. Passwords are hashed with PBKDF2 and sessions are signed,
but the traffic is plain HTTP unless you put it behind a TLS terminator.
Over the open internet, put it behind a tunnel or reverse proxy that does
TLS. On a LAN or a private tunnel it is adequate.
"""
import base64
import hashlib
import hmac
import json
import os
import secrets
import time
from pathlib import Path

# PBKDF2 rounds. High enough to make a stolen users.json expensive to
# attack, low enough that a login on a laptop is instant.
PBKDF2_ROUNDS = 260_000

SESSION_HOURS = 12

ROLES = ("viewer", "pilot")


class Users:
    """Account store backed by one JSON file next to the executable."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.secret = b""
        self._users = {}
        self._mtime = 0.0
        self._load()

    def _refresh(self):
        """Re-read the file if it changed underneath us.

        Account management runs as a separate process (`--add-user`,
        `--set-role`) so it works while a ground station is already up.
        Without this, those changes were invisible to the running server
        until it restarted: a newly created account simply could not log
        in, and revoking someone's pilot role did nothing. Checked by
        mtime, so the common case is one stat() rather than a file read.
        """
        try:
            m = self.path.stat().st_mtime
        except OSError:
            return
        if m != self._mtime:
            self._load()

    def _load(self):
        try:
            self._mtime = self.path.stat().st_mtime
        except OSError:
            self._mtime = 0.0
        if self.path.is_file():
            data = json.loads(self.path.read_text(encoding="utf-8"))
            self._users = data.get("users", {})
            self.secret = base64.b64decode(data.get("secret", ""))
        if not self.secret:
            # Signing key for session cookies. Persisted so that restarting
            # the ground station does not log everyone out.
            self.secret = secrets.token_bytes(32)
            self._save()

    def _save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps({
            "secret": base64.b64encode(self.secret).decode(),
            "users": self._users,
        }, indent=2), encoding="utf-8")
        try:
            self._mtime = self.path.stat().st_mtime
        except OSError:
            pass
        # Best effort on POSIX; a no-op on Windows, where the file sits in
        # the user's own profile anyway.
        try:
            os.chmod(self.path, 0o600)
        except OSError:
            pass

    # --- accounts ---------------------------------------------------
    def add(self, username: str, password: str, role: str = "viewer") -> tuple:
        username = (username or "").strip().lower()
        if not username:
            return False, "username required"
        if role not in ROLES:
            return False, f"role must be one of {', '.join(ROLES)}"
        if len(password or "") < 8:
            return False, "password must be at least 8 characters"
        if username in self._users:
            return False, "that username already exists"
        salt = secrets.token_bytes(16)
        self._users[username] = {
            "salt": base64.b64encode(salt).decode(),
            "hash": base64.b64encode(self._hash(password, salt)).decode(),
            "role": role,
            "created": time.time(),
        }
        self._save()
        return True, f"created {username} as {role}"

    def set_role(self, username: str, role: str) -> tuple:
        username = (username or "").strip().lower()
        if username not in self._users:
            return False, "no such user"
        if role not in ROLES:
            return False, f"role must be one of {', '.join(ROLES)}"
        self._users[username]["role"] = role
        self._save()
        return True, f"{username} is now {role}"

    def remove(self, username: str) -> tuple:
        username = (username or "").strip().lower()
        if self._users.pop(username, None) is None:
            return False, "no such user"
        self._save()
        return True, f"removed {username}"

    def list(self) -> list:
        self._refresh()
        return [{"username": u, "role": d.get("role", "viewer")}
                for u, d in sorted(self._users.items())]

    @property
    def empty(self) -> bool:
        return not self._users

    # --- authentication ---------------------------------------------
    def _hash(self, password: str, salt: bytes) -> bytes:
        return hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, PBKDF2_ROUNDS)

    def check(self, username: str, password: str):
        """Return the role on success, else None."""
        self._refresh()
        u = self._users.get((username or "").strip().lower())
        if not u:
            # Hash anyway. Returning early for an unknown username makes
            # the response measurably faster and leaks which names exist.
            self._hash(password or "", b"decoy-salt-000000")
            return None
        expected = base64.b64decode(u["hash"])
        got = self._hash(password or "", base64.b64decode(u["salt"]))
        if hmac.compare_digest(expected, got):
            return u.get("role", "viewer")
        return None

    # --- sessions ---------------------------------------------------
    def issue(self, username: str, role: str) -> str:
        """A signed, self-contained session token. No server-side session
        table, so it survives a restart and needs no cleanup."""
        payload = f"{username}|{role}|{int(time.time()) + SESSION_HOURS * 3600}"
        sig = hmac.new(self.secret, payload.encode(), hashlib.sha256).hexdigest()[:32]
        return base64.urlsafe_b64encode(f"{payload}|{sig}".encode()).decode()

    def verify(self, token: str):
        """Return {'username','role'} for a valid token, else None."""
        if not token:
            return None
        try:
            raw = base64.urlsafe_b64decode(token.encode()).decode()
            username, role, expiry, sig = raw.split("|")
        except Exception:
            return None
        payload = f"{username}|{role}|{expiry}"
        want = hmac.new(self.secret, payload.encode(), hashlib.sha256).hexdigest()[:32]
        if not hmac.compare_digest(want, sig):
            return None
        if int(expiry) < time.time():
            return None
        # The role is re-read from the store rather than trusted from the
        # token, so revoking someone's pilot access takes effect at once
        # instead of whenever their session happens to expire.
        self._refresh()
        current = self._users.get(username)
        if not current:
            return None
        return {"username": username, "role": current.get("role", "viewer")}
