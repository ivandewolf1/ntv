# Nano Templated Vector Library

This simple vector/matrix library is designed to be header only with no dependencies.

**This was written with the goals of brevity and ease of use, there is no error checking, focus on utility. Brevity > features.**

the goal was to keep each header under 100 lines of code. It's close to successful by this measure.

## compiling
most applications will not need to compile anything here, as all code is in header files. Simply #include the header, and compile your project.
The vectors library in nvec3.h do not require the other headers, this may be the only file you need.
the matrix3 and matrix4 headers require the the vector3 header, but neither requires the other matrix header.

The repository includes a simple test app, which can be compiled using cmake.

```
mkdir build
cd build
cmake ../src/
make
./tester >../testerOutput.txt
```
read tester.C for details.


## origin
There are several small vector/matrix libraries to choose from. I have previously found the one presented on parges 534 to 557 of the book "Graphics Gems IV", Jean-François Doué to be a good example
(http://www.realtimerendering.com/resources/GraphicsGems/gems.html#gemsiv)
(http://www.realtimerendering.com/resources/GraphicsGems/gemsiv/vec_mat/)

this was very brief and useful, but I needeed a version that was templated and had a determinant function, and I could not quickly find one, so I wrote this one.

I hope it is useful to you.
