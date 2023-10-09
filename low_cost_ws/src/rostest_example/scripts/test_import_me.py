import pytest
import add_path

from arg_utils.import_me_if_u_can import *
from for_example.import_me_if_u_can import say_it_pytest as say_it_pytest_1

def test_say_it_from_arg_utils():
    assert say_it_pytest() == "You have successed import me! from arg_utils pkg"

def test_say_it_from_for_example():
    assert say_it_pytest_1() == "You have successed import me! from for_example pkg"

