from .Quacker import Quacker
import pytest

# test get_quack_string function
def test_get_quack_string():
    # initialize a Quacker object
    quacker = Quacker()
    # test the get_quack_string function
    assert quacker.get_quack_string(1) == "Quack!"


