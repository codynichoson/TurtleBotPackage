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
   3. Pros -
      Cons -

   - Which of the methods would you implement and why?

   I implemented option #2 because the normalize function did not rely on any class variables, and keeping it separate
   from the class simplified the structure a little bit.

2. What is the difference between a class and a struct in C++?

Primarily, in a class, members are private by default. In a structure, members are public by default. Class variables can also have null values, whereas structure members cannot.

3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?



4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

Under the "Classes" section in the C++ guidelines, point 3 states that single-argument constructors should be declared explicit to there will not be an implicit conversion.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

# Sample Run of frame_main
```
<Put the output of a representative run here>
```