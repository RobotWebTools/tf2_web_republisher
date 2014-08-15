^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_web_republisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2014-08-15)
------------------
* release prepare
* Merge pull request #14 from T045T/send_initial_transformation
  make sure at least one transformation is sent
* make sure at least one transformation is sent, even if the tf is (and stays) the identity
* Merge pull request #13 from T045T/develop
  stop sending TF updates when removing a cancelled goal
* stop sending TF updates when removing a cancelled goal
* URLs added to package XML
* Contributors: Nils Berg, Russell Toris

0.2.1 (2014-04-08)
------------------
* cleanup
* ROS version checked for tf2 API namespace
* Contributors: Russell Toris

0.2.0 (2013-08-29)
------------------
* General clean up
* Use new tf2_ros namespace in hydro
* Added CI with travis-ci

0.1.0 (2013-02-13)
------------------
* version 0.1.0
* Merge branch 'groovy-devel' of git://github.com/KaijenHsiao/tf2_web_republisher into groovy-devel
  Conflicts:
  CMakeLists.txt
* more debug output
* merge
* Merge branch 'groovy-devel' of github.com:RobotWebTools/tf2_web_republisher into groovy-devel
  Conflicts:
  CMakeLists.txt
  src/tf_web_republisher.cpp
* catkin fixes
* added unit test files and fixed CMakeLists.txt
* added test from fuerte-devel
* updated CMakeLists to fix test_subscription2
* Merge pull request `#3 <https://github.com/RobotWebTools/tf2_web_republisher/issues/3>`_ from jkammerl/groovy-devel
  adding rate control parameter to action goal
* adding rate control per action client
* Merge branch 'fuerte-devel' of github.com:RobotWebTools/tf2_web_republisher into groovy-devel
* catkin fixes
* added function to remove leading slash in tf frame ids to avoid tf2 error
* deleting Makefile
* deleting manifest.xml
* catkinization of tf2_web_republisher.. not yet working due to missing geometry_experimental dependency
* revised version that handles multiple goals/actions
* initial commit
