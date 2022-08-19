# This file describes how to build something the ros2 packages in this folder! 

## General

In order to run anything ros2, you will need to use the visual studio command prompt. Search (windows key) and open either:

- Developer Command Prompt for VS 2019
- x64 Native Tools Command Prompt for VS 2019


In order to do anything ROS2 you will need to source the local_setup.bat which is in the foxy install location (\dev\ros2-foxy). So in the Command Prompt run the following to source and go to the ros workspace. 

```
call \dev\ros2-foxy\local_setup.bat
cd \dev\ros2_ws
```

> Note: Command Prompt is awful, 

## Colcon build

To build the workspace, run the following:

```
colcon build --symlink-install --merge-install
```

In order to enable proper logging, you will need to run with an extra command:

```
colcon build --symlink-install --merge-install --event-handlers console_cohesion+
```

All 3:
```
call \dev\ros2-foxy\local_setup.bat
cd \dev\ros2_ws
colcon build --symlink-install --merge-install --event-handlers console_cohesion+
```

## FAQ

1. Cannot create symbolic link:
    - Delete the install/ build/ and log/ folders from ros2_ws. Problem with linking. 

### Building BOOST

First you need to find an install a version of boost which is compatible with your version of python. 

Building boost can be a massive pain. Turns out it is not a header only library and you may need to manually build some of the libraries after installation. The following must be built after installation:

- Boost.Chrono
- Boost.Context
- Boost.Filesystem
- Boost.GraphParallel
- Boost.IOStreams
- Boost.Locale
- Boost.MPI
- Boost.ProgramOptions
- Boost.Python (see the Boost.Python build documentation before building and installing it)
- Boost.Regex
- Boost.Serialization
- Boost.Signals
- Boost.System
- Boost.Thread
- Boost.Timer
- Boost.Wave

In order to build these you need to open a command prompt into the boost installation dir (`C:\local\boost_1_73_0`) and run `boostrap.bat`. This inaitialises the build system. 

Then you need to run the `b1` command to build things. So e.g. to build Thread, you need to run `b1 --with-thread` 

You will then need to set the following env vars:

- BOOST_INCLUDE_DIRS=C:\local\boost_1_73_0
- BOOST_LIBRARY_DIRS=C:\local\boost_1_73_0\stage\lib