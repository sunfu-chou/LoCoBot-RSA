import pytest
import add_path

from for_example.import_me_if_u_can import say_it_works as say_it_pytest

def test_say_it_from_rostest():
    assert say_it_pytest() == "It works!"
