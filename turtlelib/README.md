# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   1. Include a member function inside a struct or class
   2. Include as its own stand alone function
   3. Create an entire class based on normalized

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   1. Pros - Elements within are easier to access
      Cons - Classes are hard
   2. Pros - Don't have to worry about class syntax
      Cons - Can't access any class variables
   3. Pros - Normalize is easily accessible everywhere within
      Cons - Overcomplicated; requires the creation of class variables and member functions just to normalize

   - Which of the methods would you implement and why?

   I implemented option #2 because the normalize function did not rely on any class variables, and keeping it separate
   from the class simplified the structure a little bit.

2. What is the difference between a class and a struct in C++?

Primarily, in a class, members are private by default. In a structure, members are public by default. Class variables can also have null values, whereas structure members cannot.

3. Why is Vector2D a struct and Transform2D a class (refer to at least 2 specific C++ core guidelines in your answer)?

Transform2D is a class because it contains an invariant (C++ Core Guidelines C.2). The functions for Vector2D do not need direct access to the representation of the class, so it is a struct (C++ Core Guidelines C.4).

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

Under the "Classes" section in the C++ guidelines, point 3 states that single-argument constructors should be declared explicit to there will not be an implicit conversion.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

Transform2D::inv() is declared const because it does not change the object itself, it returns a different result. Transform2D::operator*=(), however, changes and returns the actual object itself. According to C++ Core Guidelines Con.1, you should only make objects non-const when there is a need to change its value (as is the case with operator*=).

# Sample Run of frame_main
```
Enter transform T_{a,b}: 
deg: 90 x: 0 y: 1
Enter transform T_{b,c}: 
deg: 90 x: 1 y: 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b: 
1 1
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b: 
1 1 1
V_a [1 0 1]
V_b [1 1 1]
V_c [1 2 -1]
```