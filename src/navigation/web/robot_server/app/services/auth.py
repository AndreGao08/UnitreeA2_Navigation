import base64
import hashlib
import hmac
import json
import os
import secrets
import time

from robot_server.app.paths import AUTH_CONFIG


class AuthManager:
    def __init__(self, path=None):
        self.path = str(path or AUTH_CONFIG)
        self._ensure_config()
        with open(self.path, "r", encoding="utf-8") as stream:
            self.config = json.load(stream)

    def _ensure_config(self):
        if os.path.isfile(self.path):
            return
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        config = {
            "username": "admin",
            "login": self._password_record("123456"),
            "advanced": self._password_record("123456"),
            "session_secret": secrets.token_hex(32),
        }
        with open(self.path, "w", encoding="utf-8") as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.chmod(self.path, 0o600)

    @staticmethod
    def _password_record(password):
        salt = secrets.token_hex(16)
        digest = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt), 200000)
        return {"salt": salt, "digest": digest.hex()}

    @staticmethod
    def _verify_password(password, record):
        digest = hashlib.pbkdf2_hmac(
            "sha256", str(password).encode(), bytes.fromhex(record["salt"]), 200000
        ).hex()
        return hmac.compare_digest(digest, record["digest"])

    def verify_login(self, username, password):
        expected_user = os.getenv("HYY_ADMIN_USERNAME", self.config["username"])
        env_password = os.getenv("HYY_ADMIN_PASSWORD")
        valid_password = (
            hmac.compare_digest(str(password), env_password)
            if env_password is not None
            else self._verify_password(password, self.config["login"])
        )
        return hmac.compare_digest(str(username), expected_user) and valid_password

    def verify_advanced(self, password):
        env_password = os.getenv("HYY_ADVANCED_PASSWORD")
        if env_password is not None:
            return hmac.compare_digest(str(password), env_password)
        return self._verify_password(password, self.config["advanced"])

    def create_session(self, username, lifetime=8 * 3600):
        payload = f"{username}|{int(time.time()) + lifetime}"
        encoded = base64.urlsafe_b64encode(payload.encode()).decode().rstrip("=")
        signature = hmac.new(
            self.config["session_secret"].encode(), encoded.encode(), hashlib.sha256
        ).hexdigest()
        return f"{encoded}.{signature}"

    def valid_session(self, token):
        try:
            encoded, signature = str(token).split(".", 1)
            expected = hmac.new(
                self.config["session_secret"].encode(), encoded.encode(), hashlib.sha256
            ).hexdigest()
            if not hmac.compare_digest(signature, expected):
                return False
            padded = encoded + "=" * (-len(encoded) % 4)
            username, expires = base64.urlsafe_b64decode(padded).decode().split("|", 1)
            expected_user = os.getenv("HYY_ADMIN_USERNAME", self.config["username"])
            return hmac.compare_digest(username, expected_user) and int(expires) >= time.time()
        # Cookies are untrusted input; malformed base64/Unicode or a damaged
        # payload must be treated as logged out instead of surfacing a 500.
        except (ValueError, TypeError, UnicodeError, KeyError):
            return False


auth_manager = AuthManager()
