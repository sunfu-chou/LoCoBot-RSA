import pytest

from .Quacker import Quacker

# test get_quack_string function
def test_get_quack_string():
    # initialize a Quacker object
    quacker = Quacker()
    # test the get_quack_string function
    assert quacker.get_quack_string(1) == "Quack!"

# test rounded mean funcition
def test_rounded_mean():
    # initialize a Quacker object
    quacker = Quacker()
    # test the rounded_mean function
    assert quacker.rounded_mean([1, 2, 3]) == 2
