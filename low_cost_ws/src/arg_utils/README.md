# arg_utils
## Python package mangement in ros package example
Put your python moudle into /include/for_example/ or new a folder inside include.

Add a add_path.py where your main code want to be.

Then code like this...

<img src="./image/add_path_example.png"/>

If you want to new a python module, just add the following code to your add_path.py
```
sys.path.append(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '<path>'))
```
and make sure your < path > is pointing correctly to your python package.
### testing python package
after runing the docker
```
$ cd ~/LoCoBot-RSA/low_cost_ws/src/rostest_example/scripts/
$ pytest test_import_me.py
$ python3 testing_pypkg_from_arg_utils.py
```
