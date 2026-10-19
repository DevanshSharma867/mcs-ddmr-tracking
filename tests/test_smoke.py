"""Phase 0 smoke test: the package imports and the Controller interface is abstract."""

import pytest

from ddmr.controllers.base import Controller


def test_controller_is_abstract() -> None:
    with pytest.raises(TypeError):
        Controller()  # type: ignore[abstract]
