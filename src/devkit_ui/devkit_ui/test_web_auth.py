"""The dashboard password gate must refuse every request that does not carry the shared password."""
import base64
import unittest

from starlette.applications import Starlette
from starlette.responses import PlainTextResponse
from starlette.routing import Route, WebSocketRoute
from starlette.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from devkit_ui.web_auth import BasicAuthMiddleware


def _basic(user: str, password: str) -> dict[str, str]:
    """Build an Authorization header for the given credentials."""
    return {'Authorization': 'Basic ' + base64.b64encode(f'{user}:{password}'.encode()).decode()}


async def _index(_request):
    """Serve a marker page."""
    return PlainTextResponse('dashboard')


async def _ws(websocket):
    """Accept and greet, so a refused handshake is distinguishable."""
    await websocket.accept()
    await websocket.send_text('hello')
    await websocket.close()


class BasicAuthMiddlewareTest(unittest.TestCase):
    """Behaviour of the shared-password gate."""

    def setUp(self) -> None:
        """Serve a page and a websocket behind the gate."""
        app = Starlette(routes=[Route('/', _index), WebSocketRoute('/ws', _ws)])
        app.add_middleware(BasicAuthMiddleware, password='s3cret')
        self.client = TestClient(app)

    def test_missing_credentials_get_401_with_a_challenge(self):
        """Verify an unauthenticated request is refused and the browser is asked to log in."""
        response = self.client.get('/')
        self.assertEqual(response.status_code, 401)
        self.assertIn('Basic', response.headers['www-authenticate'])
        self.assertNotIn('dashboard', response.text)

    def test_wrong_password_is_refused(self):
        """Verify a wrong password is refused regardless of user name."""
        self.assertEqual(self.client.get('/', headers=_basic('x', 'nope')).status_code, 401)

    def test_correct_password_passes_with_any_user_name(self):
        """Verify the user name is ignored and a colon inside the password is allowed."""
        self.assertEqual(self.client.get('/', headers=_basic('anyone', 's3cret')).text, 'dashboard')
        app = Starlette(routes=[Route('/', _index)])
        app.add_middleware(BasicAuthMiddleware, password='a:b')
        self.assertEqual(TestClient(app).get('/', headers=_basic('u', 'a:b')).status_code, 200)

    def test_malformed_headers_are_refused(self):
        """Verify non-Basic schemes and bad base64 are refused rather than raising."""
        for value in ('Bearer s3cret', 'Basic !!!notbase64', 'Basic', 'Basic ' + base64.b64encode(b's3cret').decode()):
            with self.subTest(value=value):
                self.assertEqual(self.client.get('/', headers={'Authorization': value}).status_code, 401)

    def test_websocket_without_credentials_is_refused(self):
        """Verify the joystick websocket cannot be reached without the password."""
        with self.assertRaises(WebSocketDisconnect), self.client.websocket_connect('/ws'):
            pass

    def test_websocket_with_credentials_connects(self):
        """Verify the websocket works once the password is supplied."""
        with self.client.websocket_connect('/ws', headers=_basic('u', 's3cret')) as ws:
            self.assertEqual(ws.receive_text(), 'hello')

    def test_empty_password_is_rejected_at_construction(self):
        """Verify an empty password cannot silently disable the gate."""
        with self.assertRaises(ValueError):
            BasicAuthMiddleware(lambda *_: None, password='')


if __name__ == '__main__':
    unittest.main()
