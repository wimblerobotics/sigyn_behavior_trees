# Contributing to sigyn_behavior_trees

Thank you for considering a contribution to `sigyn_behavior_trees`!

## How to Contribute

1. **Fork** the repository and create a feature branch.
2. **Follow the coding style**: this package uses Google C++ style enforced by `clang-format`.
   Run `clang-format -i` on any modified `.cpp` / `.hpp` files before committing.
3. **Write tests**: add or update unit/integration tests under `test/` for any new or changed
   behaviour.
4. **Build and test locally** before opening a pull request:

   ```bash
   cd ~/sigyn_behavior_trees_ws
   colcon build --packages-select sigyn_behavior_trees
   colcon test  --packages-select sigyn_behavior_trees
   colcon test-result --verbose
   ```

5. **Open a pull request** with a clear description of what was changed and why.

## License

All contributions are accepted under the [Apache-2.0 License](LICENSE).

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
