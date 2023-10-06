import pytest

import add_path
from rostest_example.Quacker import Quacker

# test Quacker rounded_mean function with a list of integers
def test_quacker_rounded_mean_int():
    q = Quacker()
    assert q.rounded_mean([1,2,3,4,5]) == 3



