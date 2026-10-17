"""Optional shared-password gate for the dashboard.

The dashboard serves a joystick and a soft e-stop on port 80 of a robot that is usually reached over
Wi-Fi. Without a password, anyone on that network can drive it. Set SOWBOT_UI_PASSWORD (for example in
.env) to require HTTP Basic credentials on every request, WebSocket handshakes included. The user name
is ignored. HTTP Basic is not encrypted: this keeps casual and accidental access out, it is not a
substitute for a private network or a VPN.
"""

import base64
import binascii
import hmac

UI_PASSWORD_ENV = 'SOWBOT_UI_PASSWORD'

_REALM = b'Basic realm="Sowbot dashboard", charset="UTF-8"'


class BasicAuthMiddleware:
    """Pure ASGI middleware that rejects http and websocket requests without the shared password."""

    def __init__(self, app, password: str) -> None:
        """Wrap an ASGI app with a shared password; raise ValueError if the password is empty."""
        if not password:
            raise ValueError('password must not be empty')
        self._app = app
        self._password = password.encode()

    async def __call__(self, scope, receive, send) -> None:
        """Pass authorized HTTP/WebSocket requests and other ASGI scope types to the wrapped app.

        Reject unauthorized HTTP requests with a 401 Basic challenge and WebSocket handshakes
        with close code 1008. Errors from the wrapped app, receive, or send propagate.
        """
        if scope['type'] not in ('http', 'websocket'):
            await self._app(scope, receive, send)
            return
        if self._authorised(scope.get('headers', [])):
            await self._app(scope, receive, send)
            return
        if scope['type'] == 'websocket':
            await receive()  # consume websocket.connect before refusing it
            await send({'type': 'websocket.close', 'code': 1008})
            return
        await send({'type': 'http.response.start', 'status': 401, 'headers': [
            (b'www-authenticate', _REALM), (b'content-type', b'text/plain; charset=utf-8'),
            (b'content-length', b'12')]})
        await send({'type': 'http.response.body', 'body': b'Unauthorised'})

    def _authorised(self, headers) -> bool:
        """Check the first Authorization header for Basic credentials with the shared password.

        Ignore the username; passwords may contain colons. Return False for a missing header,
        a non-Basic scheme, invalid base64, or a password mismatch.
        """
        for name, value in headers:
            if name.lower() != b'authorization':
                continue
            scheme, _, token = value.partition(b' ')
            if scheme.lower() != b'basic':
                return False
            try:
                _, _, supplied = base64.b64decode(token.strip(), validate=True).partition(b':')
            except (binascii.Error, ValueError):
                return False
            return hmac.compare_digest(supplied, self._password)
        return False
