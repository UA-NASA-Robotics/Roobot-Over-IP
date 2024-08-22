# UnityTypes

Multi-Dimensional Variables for multi-dimensional data.
Unified representation of multi-dimensional data in a single variable for
the University of Akron Nasa Mining Competition Robotics Team.

## Usage

Use these types to store dimensional data in a single variable.
For example, you can store a 3D position in a float3 variable.

Define them as you would any other variable, and use them as you would any other variable.

`float3 position = {1, 2, 3};` Constructs a 3D position at (1, 2, 3).

`float2 position2 = position;` Converts the 3D position to a 2D position at (1, 2).

`float3 position3 = position + position2;` Adds the 3D and 2D positions together, resulting in a 3D position at (2, 4, 3).

These types are designed to be easy to use and intuitive, but it is recommended to note that some operations may not be supported or different because of how vectors work.

## Vector Math

These types all have overloaded operators to make them easy to use, and align with common vector
operations. Note some irregularities in the operators, to align with vector operations.

1.  Cross/Dot Product:

    - Default is cross product when using the \* operator, but dot product is also available as a function: `.dot()`

2.  Magnitude:

    - The magnitude of a vector is known as the modulus in vector math. Can be accessed with `.magnitude()`.

3.  Absolute Value:

    - The absolute value of a vector is the vector with all positive components. Can be accessed with the
      unary plus operator: +

4.  Negation/Inversion:

    - The negation of a vector is the vector with all components negated. Can be accessed with the unary
      minus operator: -

5.  Truncation:

    - In the event of a vector being converted to a a lower dimension, the higher dimension components are
      truncated. For example, a vector3 being converted to a vector2 will lose the z component.

    - If a `vector3 * vector2` operation is performed, the z component is ignored. Same for `vector4 * vector3` and `vector4 * vector2`. The higher dimension components are ignored.
      However, if a `vector3 * scalar` operation is performed, the scalar is multiplied by all components.

6.  Ascending Dimensional Operations

    - There is no `vector2 * vector3` operation, as it is redundant. The ideal operation is `vector3 * vector2`. If you need to multiply a vector2 by a
      vector3, you can convert the vector2 to a vector3 first, and afterwards truncate the result.

## Names and Namespaces

The names of these types are designed to be easy to understand and use. They are named after the number of dimensions they have, and the type of data they hold. These types are stored in the `UnityTypes` namespace.
Definite types are also included in this list, where an extra \_\* is added to the end of the type name. These types are used especially for embedded systems, where the size of the data is important.

```cpp
#include "UnityTypes.h"

using namespace unity;

int main() {
    float3 position = {1, 2, 3}; // A 3D position at (1, 2, 3).

    int3 dimensions = {1, 2, 3};

    uint4 color = {255, 255, 255, 255}; //A 4D color with RGBA values of 255. Uses general unsigned integers.

    uint8_t_4 color8 = color; //A 4D color with RGBA values of 255. Uses definite 8-bit unsigned integers.
}
```

While almost all of the possible data types are implemented, there is potential for more types to be implemented in the future as typedefs. If a new type is needed, it can be added to the `UnityTypes` namespace via `typedefgen.py` which generates a list of typedef. For quick access, a `Vector*<T>` template can be called to generate a new type of vector dimensions \* and of type T. Currently dimensions 2-4 are implemented. 1D vectors are not implemented, as they are should be represented as a scalar standard variable.

## Library Use

This is a header-only library, so it can be included in any project by including the header file. The header file is located in the `lib` directory, along with this readme.

To include the library in your project, include the header file in your source file. It is recommended to use the namespace `unity` to access the types without having to type `unity::` before each type.

```cpp
#include "UnityTypes.h"

using namespace unity;

int main() {
    float3 position = {1, 2, 3};
    float2 position2 = position;
    float3 position3 = position2;

    return position.dot(position2); // Returns the dot product of the two vectors.
}
```

### To Be Implemented

- Logical Operators: `&&` and `||` operators are not implemented yet, but are planned to be implemented in the future. Serious consideration is being given to the implementation of these operators, as they are not common in vector math, and may be redundant or purposeless.
