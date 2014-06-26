#include <stdio.h>
#include <math.h>


#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* min finds the lowest of four doubles */
inline double min(double a, double b, double c, double d) {
    return MIN(a, MIN(b, MIN(c, d)));
}

/* max finds the highest of four doubles */
inline double max(double a, double b, double c, double d) {
    return MAX(a, MAX(b, MAX(c, d)));
}

/* swap simply swaps the stored values of two doubles */
void swap(double *a, double *b) {
    double tmp = *a;
    *a = *b;
    *b = tmp;
}

/* sort takes the coordinates of two different points and orders them,
   first from left to right and (if one is above another) from bottom to top */
void sort(double *x1, double *x2, double *y1, double *y2) {
    if (*x1 <= *x2 && *y1 <= *y2) return;
    if (*x1 > *x2) {
        swap(x1, x2);
        swap(y1, y2);
    }
    else if (*x1 == *x2) {
        swap(y1, y2);
    }
}

/* a point is made from a pair of x,y coordinates */
typedef struct point {
    double x;
    double y;
} point;

/* a line is made from a pair of points,
   at this stage it is assumed that all lines are straight */
typedef struct line {
    struct point pt1;
    struct point pt2;
} line;

//typedef point line[2];

/* a vector is made from its x and y components */
typedef struct vector {
    double x;
    double y;
} vector;

//typedef double vector[2];

/* findX takes a given line and y coordinate and returns the corresponding x coordinate,
   this can only work for non-horizontal lines and should not be called otherwise,
   the basic formula used is x = (C - B*y) / A, where A, B and C are doubles*/
inline double findX(double y, line l) {
    double D = l.pt1.x - l.pt2.x;
    double E = l.pt2.y - l.pt1.y;
    //return ((l.pt2.y - l.pt1.y)*l.pt1.x + (l.pt1.x - l.pt2.x)*l.pt1.y - (l.pt1.x - l.pt2.x)*y) / (l.pt2.y - l.pt1.y);
    //return (E*l.pt1.x - D*l.pt1.y - D*y) / E;
    //return (E*l.pt1.x - D*(l.pt1.y + y)) / E;
    return l.pt1.x - (D / E)*(l.pt1.y + y);
}

/* findY takes a given line and x coordinate and returns the corresponding y coordinate,
   this can only work for non-vertical lines and should not be called otherwise,
   the basic formula used is y = (C - A*x) / B, where A, B and C are doubles*/
 inline double findY(double x, line l) {
    double D = l.pt1.x - l.pt2.x;
    double E = l.pt2.y - l.pt1.y;
    //return ((l.pt2.y - l.pt1.y)*l.pt1.x + (l.pt1.x - l.pt2.x)*l.pt1.y - (l.pt2.y - l.pt1.y)*x) / (l.pt1.x - l.pt2.x);
    //return (E*l.pt1.x + D*l.pt1.y - E*x) / D;
    //return (E*(l.pt1.x - x) + D*l.pt1.y) / D;
    return (E / D)*(l.pt1.x - x) + l.pt1.y;
}

/* det finds the relation between a given vector and a line,
   this is used to help to find a point of intersection,
   if the result is zero then the two are parallel */
inline double det(vector v, line e) {
    return (v.y*(e.pt1.x - e.pt2.x)) - (v.x*(e.pt1.y - e.pt2.y));
}

/* intersection finds a point where a given vector starting
   from a given point intersects with a particular line,
   if there is no intersection then it will return null */
point intersection(point I, vector v, line e) {
    double d = det(v, e);
    printf("det = %f\n", d);
    double ex = e.pt1.x - e.pt2.x;
    double ey = e.pt2.y - e.pt1.y;
    double Iv = v.y*I.x - v.x*I.y;
    double exy = ey*e.pt1.x + ex*e.pt1.y;
    if (d != 0) {
        double x = (ex*Iv + v.x*exy) / d;
        double y = (v.y*exy - ey*Iv) / d;
        point p = {x, y};
        return p;
    }
    point p = { INFINITY, INFINITY };
    return p;
}

/* intervalRange finds a range on a projected line of infinite length which
   both satisfies the same linear equation as a given line and also is
   reachable to a trajectory from a given line and a pair of given vectors,
   the projected line is not the same as the edge as it is not bounded by
   any end points, rather the edge is a segment of this line */
line intervalRange(line I, vector a, vector b, line e) {
    point i1a = intersection(I.pt1, a, e);
    point i1b = intersection(I.pt1, b, e);
    point i2a = intersection(I.pt2, a, e);
    point i2b = intersection(I.pt2, b, e);
    double xmin = min(i1a.x, i1b.x, i2a.x, i2b.x);
    double ymin = min(i1a.y, i1b.y, i2a.y, i2b.y);
    if (i1a.x == INFINITY) {point i1a = { -INFINITY, -INFINITY }; }
    if (i1b.x == INFINITY) {point i1b = { -INFINITY, -INFINITY }; }
    if (i2a.x == INFINITY) {point i2a = { -INFINITY, -INFINITY }; }
    if (i2b.x == INFINITY) {point i2b = { -INFINITY, -INFINITY }; }
    double xmax = max(i1a.x, i1b.x, i2a.x, i2b.x);
    double ymax = max(i1a.y, i1b.y, i2a.y, i2b.y);
    line l;
    if (xmin < xmax) {
        l.pt1.x = xmin;
        l.pt1.y = findY(xmin, e);
        l.pt2.x = xmax;
        l.pt2.y = findY(xmax, e);
        return l;
    }
    l.pt1.x = findX(ymin, e);
    l.pt1.y = ymin;
    l.pt2.x = findX(ymax, e);
    l.pt2.y = ymax;
    return l;
}

/* range finds the intersection of the line found from
   intervalRange and a given line, it is the final step
   of finding the line of reachability of a given edge */
line range(line I, line e) {
    line l;
    if (I.pt1.x < I.pt2.x) {
        l.pt1.x = MAX(I.pt1.x, e.pt1.x);
        l.pt1.y = findY(l.pt1.x, I);
        l.pt2.x = MIN(I.pt2.x, e.pt2.x);
        l.pt2.y = findY(l.pt2.x, I);
        return l;
    }
    l.pt1.y = MAX(I.pt1.y, e.pt1.y);
    l.pt1.x = findX(l.pt1.y, I);
    l.pt2.y = MIN(I.pt2.y, e.pt2.y);
    l.pt2.x = findX(l.pt2.y, I);
    return l;
}

/* reachability prints the reachable range of a given edge from
   a given starting Interval and a given pair of vectors, in a
   neat way and prints a statement noting if it is unreachable */
void reachability(line I, vector a, vector b, line e) {
    line ir = intervalRange(I, a, b, e);
    printf("interval range is (%f, %f) to (%f, %f)\n", ir.pt1.x, ir.pt1.y, ir.pt2.x, ir.pt2.y);
    printf("edge is (%f, %f) to (%f, %f)\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y);
    if (((ir.pt1.x != ir.pt2.x) && (ir.pt1.x > e.pt2.x || ir.pt2.x < e.pt1.x)) || ((ir.pt1.x == ir.pt2.x) && (ir.pt1.y > e.pt2.y || ir.pt2.y < e.pt1.y))) {
        printf("the edge is unreachable\n");
    }
    else{
        line r = range(ir, e);
        printf("the edge from (%f, %f) to (%f, %f) is reachable from (%f, %f) to (%f, %f)\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y, r.pt1.x, r.pt1.y, r.pt2.x, r.pt2.y);
    }
}

main()
{
    double I1x, I1y, I2x, I2y, ax, ay, bx, by, e1x, e1y, e2x, e2y;

    struct point I1;
    struct point I2;
    struct line I;
    struct vector a;
    struct vector b;
    struct point e1;
    struct point e2;
    struct line e;

    printf("enter I1x, I1y, I2x, I2y, ax, ay, bx, by, e1x, e1y, e2x, e2y\n");
    scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &(I.pt1.x), &(I.pt1.y), &(I.pt2.x), &(I.pt2.y), &(a.x), &(a.y), &(b.x), &(b.y), &(e.pt1.x), &(e.pt1.y), &(e.pt2.x), &(e.pt2.y));

    sort(&I1x, &I2x, &I1y, &I2y);
    sort(&e1x, &e2x, &e1y, &e2y);

    reachability(I, a, b, e);

    return 0;
}
