package de.tu_chemnitz.projecttangostudy;


/**
 *  ========================================================
 *  Vector2D.java: Source code for two-dimensional vectors
 *
 *  Written by: Mark Austin                   November, 2005
 *  ========================================================
 */

import java.lang.Math;

public class Vec2D {

    protected double dX;
    protected double dY;

    // Constructor methods ....

    public Vec2D() {
        dX = dY = 0.0;
    }

    public Vec2D( double dX, double dY ) {
        this.dX = dX;
        this.dY = dY;
    }

    // Convert vector to a string ...

    public String toString() {
        return "Vec2D(" + dX + ", " + dY + ")";
    }

    // Compute magnitude of vector ....

    public double length() {
        return Math.sqrt ( dX*dX + dY*dY );
    }

    // Sum of two vectors ....

    public Vec2D add( Vec2D v1 ) {
        Vec2D v2 = new Vec2D( this.dX + v1.dX, this.dY + v1.dY );
        return v2;
    }

    // Subtract vector v1 from v .....

    public Vec2D sub( Vec2D v1 ) {
        Vec2D v2 = new Vec2D( this.dX - v1.dX, this.dY - v1.dY );
        return v2;
    }

    // Scale vector by a constant ...

    public Vec2D scale( double scaleFactor ) {
        Vec2D v2 = new Vec2D( this.dX*scaleFactor, this.dY*scaleFactor );
        return v2;
    }

    // Scale vector by a constant ...

    public Vec2D scale_fixed( double scaleFactor ) {
        Vec2D v2 = new Vec2D( scaleFactor, scaleFactor );
        return v2;
    }

    // Normalize a vectors length....

    public Vec2D normalize() {
        Vec2D v2 = new Vec2D();

        double length = Math.sqrt( this.dX*this.dX + this.dY*this.dY );
        if (length != 0) {
            v2.dX = this.dX/length;
            v2.dY = this.dY/length;
        }

        return v2;
    }

    // Dot product of two vectors .....

    public double dotProduct ( Vec2D v1 ) {
        return this.dX*v1.dX + this.dY*v1.dY;
    }

    // Exercise methods in Vec2D class

    public static void main ( String args[] ) {
        Vec2D vA = new Vec2D( 1.0, 2.0);
        Vec2D vB = new Vec2D( 2.0, 2.0);

        System.out.println( "Vector vA =" + vA.toString() );
        System.out.println( "Vector vB =" + vB.toString() );

        System.out.println( "Vector vA-vB =" + vA.sub(vB).toString() );
        System.out.println( "Vector vB-vA =" + vB.sub(vA).toString() );

        System.out.println( "vA.normalize() =" + vA.normalize().toString() );
        System.out.println( "vB.normalize() =" + vB.normalize().toString() );

        System.out.println( "Dot product vA.vB =" + vA.dotProduct(vB) );
        System.out.println( "Dot product vB.vA =" + vB.dotProduct(vA) );
    }
    
}
