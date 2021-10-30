# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. 

### Temporary note
If you are not working directly on the Virtual Machine, you can test implemented functions by:
- commenting in the file `CMakeLists.txt` the lines:
    - `find_package(OpenCV REQUIRED )`
    - `find_package(project_interface REQUIRED )`
- running:
    - mkdir build
    - cd build
    - cmake ..
    - make
    - ./functions-testing
