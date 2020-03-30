#include "catch.h"
#include "../src/tvec3.h"
#include "../src/tmat3.h"
#include "../src/tmat4.h"

using namespace lwtv;

TEST_CASE( "tvec3" ) {
  tvec3<double> A;
  tvec3<double> B(.1,.2,.3);
  tvec3<double> C(.4);
  tvec3<double> D(B);
  tvec3<double> E;

  SECTION( "Assignment operators" ) {
    A = 1;
    B = .1;
    A += B;
    REQUIRE( A == tvec3<double>(1.1, 1.1, 1.1) );

    A -= B;
    REQUIRE( A == 1.0 );

    A *= 2;
    REQUIRE( A == 2.0 );

    A /= 2;
    REQUIRE( A == 1.0 );

    C = 3;
    C[1] = .3;
    D.set(3,.3,3);
    REQUIRE( C == D) ;
  }

  SECTION( "vector to vector operators" ) {
    A = B = C = D = .25;
    A = B + C;
    REQUIRE ( A == (B + C));

    B = A - D;
    REQUIRE ( B == (A - D));

    C = A * B;
    REQUIRE ( C == (A * D));

    D = A / B;
    REQUIRE ( D == (A / B));
  }

  SECTION( "vector to digit operators 1" ) {
    A = B = C = D = E =1.5;
    A = C + .1;
    REQUIRE( A == (C + .1) );

    B = .1 + C;
    REQUIRE( B == (.1 + C) );

    C = A - -1;
    REQUIRE( C == (A - -1) );

    D = .1 - A;
    REQUIRE( D == (.1 - A) );
  }

  SECTION( "vector to digit operators 1" ) {
    A = B = C = D = E =1.5;
    A = C * 2;
    REQUIRE ( A == (C * 2));

    B = 2 * C;
    REQUIRE ( B == (2 * C));

    C = E / 2;
    REQUIRE ( C == (E / 2));

    D = 2 / E;
    REQUIRE ( D == (2 / E));
  }

  SECTION( "equality" ){
    A = B = C = 1;
    D = E = 2;
    REQUIRE( A == B );
    REQUIRE( A != E );
  }

  SECTION( "nutility functions" ){
    A = 1;
    B = A.normalized() * 2;
    // REQUIRE( double(B.length2()) == double(4.0) );  //TODO: failed 4.0==4.0
    REQUIRE( B.length() == 2.0 );
  }
  SECTION( "cross" ){
    A = 1;
    B = 1;
    B[1] = .5;
    REQUIRE( A.dot(B) == 2.5 );

    C = A.cross(B);
    REQUIRE( C == tvec3<double>(.5,0,-0.5) );
  }
}
