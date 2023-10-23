import lec9_inheritance as inh
import pytest

def test_animal():
    a = inh.Animal(4)
    print(a)
    print(a.get_age())
    a.set_name("fluffy")
    print(a)
    assert a.get_name() == "fluffy"
    assert a.get_age() == 4

def test_cat():
    c = inh.Cat(5)
    print(c)
    print(c.get_age())
    c.set_name("fluffy")
    print(c)
    assert c.get_name() == "fluffy"
    assert c.get_age() == 5
    print(c.speak())
    #assert c.speak() == 'meow'

def test_person():
    p = inh.Person("Captain", 30)
    print(p)
    print(p.get_age())
    p.set_name("Captain")
    print(p)
    assert p.get_name() == "Captain"
    assert p.get_age() == 30

    print("\n---- person tests ----")
    p1 = inh.Person("jack", 30)
    p2 = inh.Person("jill", 25)
    print(p1.get_name())
    print(p1.get_age())
    print(p2.get_name())
    print(p2.get_age())
    print(p1)
    p1.speak()
    p1.age_diff(p2)



