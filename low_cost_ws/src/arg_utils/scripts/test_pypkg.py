#!/usr/bin/env python3

import add_path

from arg_utils.import_me_if_u_can import say_it_works as say_it_works
from for_example.import_me_if_u_can import say_it_works as say_it_works_2

# write a test function for say_it_works
def test_say_it_works():
    assert say_it_works() == "It works!"

def test_say_it_works_2():
    assert say_it_works_2() == "It works!"
    
#say_it_works()
#say_it_works_2()
