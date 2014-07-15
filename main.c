#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <malloc.h>

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
void sort(double *x1, double *y1, double *x2, double *y2) {
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

/* a vector is made from its x and y components */
typedef struct vector {
    double x;
    double y;
} vector;

/* regionEdges are the four edges of a region */
typedef struct regionEdges {
    struct line e1;
    struct line e2;
    struct line e3;
    struct line e4;
} regionEdges;

/* regionVectors are the two vectors of a region */
typedef struct regionVectors {
    struct vector a;
    struct vector b;
} regionVectors;

/* a region is made up from its edges and vectors */
typedef struct region {
    struct regionEdges e;
    struct regionVectors v;
} region;

/* line_collection is an array used for creating a reachability tree */
line line_collection[1000];

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

/* collinearIntersection finds the intersection of the line found
   from intervalRange and a given line, it is the final step
   of finding the line of reachability of a given edge */
line collinearIntersection(line I, line e) {
    line l;
    double AI = I.pt2.y - I.pt1.y;
    double Ae = e.pt2.y - e.pt1.y;
    double BI = I.pt1.x - I.pt2.x;
    double Be = e.pt1.x - e.pt2.x;
    double CI = AI*I.pt1.x + BI*I.pt1.y;
    double Ce = Ae*e.pt1.x + Be*e.pt1.y;
    if (AI*e.pt1.x + BI*e.pt1.y - CI == Ae*e.pt1.x + Be*e.pt1.y - Ce){
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
}

/* isCollinear determines whether two line segments are collinear,
   meaning that their Ax + By = C expressions are identical, or that
   they are both discrete segments of the same line of infinite length*/
bool isCollinear(line I, line e) {
    double AI = I.pt2.y - I.pt1.y;
    double Ae = e.pt2.y - e.pt1.y;
    double BI = I.pt1.x - I.pt2.x;
    double Be = e.pt1.x - e.pt2.x;
    double CI = AI*I.pt1.x + BI*I.pt1.y;
    double Ce = Ae*e.pt1.x + Be*e.pt1.y;
    double d = AI*Be - Ae*BI;
    if (d == 0 && AI*e.pt1.x + BI*e.pt1.y - CI == Ae*e.pt1.x + Be*e.pt1.y - Ce){
        return true;
    }
    return false;
}

/* isCollinearIntersection determines whether two line segments have a
   collinear intersection, meaning that they are both collinear and that there
   is an computable line where the two line segments intersect with each other */
bool isCollinearIntersection(line I, line e) {
    double AI = I.pt2.y - I.pt1.y;
    double Ae = e.pt2.y - e.pt1.y;
    double BI = I.pt1.x - I.pt2.x;
    double Be = e.pt1.x - e.pt2.x;
    double CI = AI*I.pt1.x + BI*I.pt1.y;
    double Ce = Ae*e.pt1.x + Be*e.pt1.y;
    double d = AI*Be - Ae*BI;
    if (d == 0 && AI*e.pt1.x + BI*e.pt1.y - CI == Ae*e.pt1.x + Be*e.pt1.y - Ce && I.pt1.x >= e.pt1.x && I.pt2.x <= e.pt2.x && I.pt1.y >= e.pt1.y && I.pt2.y <= e.pt2.y){
        return true;
    }
    return false;
}

/* lineIntersection takes two lines which are neither
   parallel nor collinear with respect to each other, and
   finds the point where they intersect with each other */
point lineIntersection(line l1, line l2){
    point p;
    double A1 = l1.pt2.y - l1.pt1.y;
    double A2 = l2.pt2.y - l2.pt1.y;
    double B1 = l1.pt1.x - l1.pt2.x;
    double B2 = l2.pt1.x - l2.pt2.x;
    double C1 = A1*l1.pt1.x + B1*l1.pt1.y;
    double C2 = A2*l2.pt1.x + B2*l2.pt1.y;
    double d = A1*B2 - A2*B1;
    if (d != 0){
        p.x = ((B2*C1 - B1*C2) / d);
        p.y = ((A1*C2 - A2*C1) / d);
        return p;
    }
}

/* distance finds the length of the distance between two points */
double distance(point p1, point p2){
    double d = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    return d;
}

/* isImmediatelyReachable determines whether any part of a line is reachable
   given just one starting interval and one pair of vectors. This is useful
   for finding the edges of a region that are reachable from an entry interval */
bool isImmediatelyReachable(line I, vector a, vector b, line e) {
    line ir = intervalRange(I, a, b, e);
    if (isCollinear(I, e) || ir.pt1.x > e.pt2.x || ir.pt2.x < e.pt1.x || (ir.pt1.x == ir.pt2.x && (ir.pt1.y > e.pt2.y || ir.pt2.y < e.pt1.y)) /*|| distance(collinearIntersection(I, e).pt1, collinearIntersection(I, e).pt2) > 0*/) {
        return false;
    }
    else{
        line r = collinearIntersection(ir, e);
        if (r.pt1.x == r.pt2.x && r.pt1.y == r.pt2.y){
            return false;
        }
        else{
            return true;
        }
    }
}

/* immediateReachability finds the line segment of reachability
   of a line from an interval and a pair of vectors */
line immediateReachability(line I, vector a, vector b, line e) {
    if(isImmediatelyReachable(I, a, b, e)) {
        line ir = intervalRange(I, a, b, e);
        line r = collinearIntersection(ir, e);
        return r;
    }
}

/* immediateReachabilityReverse works the same way as immediateReachability,
   except that the directions of the vectors are reversed. This is useful
   when finding a list of preceding intervals */
line immediateReachabilityReverse(line I, vector a, vector b, line e, line eI) {
    a.x = -a.x;
    a.y = -a.y;
    b.x = -b.x;
    b.y = -b.y;
    if(isImmediatelyReachable(I, a, b, e)) {
        line ir = intervalRange(I, a, b, e);
        line r = collinearIntersection(ir, e);
        return r;
    }
}

/* presentImmediateReachability prints the reachable range of a given
   edge from a given starting Interval and a given pair of vectors,
   in a neat way and prints a statement noting if it is unreachable */
void presentImmediateReachability(line I, vector a, vector b, line e) {
    if (!isImmediatelyReachable(I, a, b, e)) {
        printf("the edge (%f, %f) to (%f, %f) is unreachable\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y);
    }
    else {
        line r = immediateReachability(I, a, b, e);
        printf("the edge from (%f, %f) to (%f, %f) is reachable from (%f, %f) to (%f, %f)\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y, r.pt1.x, r.pt1.y, r.pt2.x, r.pt2.y);
    }
    //line ir = intervalRange(I, a, b, e);
    /*if ((I.pt1.x == e.pt1.x && I.pt1.y == e.pt1.y && I.pt2.x == e.pt2.x && I.pt2.y == e.pt2.y) || ir.pt1.x > e.pt2.x || ir.pt2.x < e.pt1.x || (ir.pt1.x == ir.pt2.x && (ir.pt1.y > e.pt2.y || ir.pt2.y < e.pt1.y))) {
        printf("the edge (%f, %f) to (%f, %f) is unreachable\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y);
    }
    else{
        line r = range(ir, e);
        if (r.pt1.x == r.pt2.x && r.pt1.y == r.pt2.y){
            printf("the edge (%f, %f) to (%f, %f) is unreachable\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y);
        }
        else{
            printf("the edge from (%f, %f) to (%f, %f) is reachable from (%f, %f) to (%f, %f)\n", e.pt1.x, e.pt1.y, e.pt2.x, e.pt2.y, r.pt1.x, r.pt1.y, r.pt2.x, r.pt2.y);
        }
    }*/
}

/* hasEdgeWithLineSegment determines (sort of) whether a region has an edge
   containing a particular line segment, and if it does it returns the edge */
line hasEdgeWithLineSegment(region R, line l){
    /*line cl = collinearIntersection(l, R.e.e1);
    if (cl.pt1.x == l.pt1.x && cl.pt1.y == l.pt1.y && cl.pt2.x == l.pt2.x && cl.pt2.y == l.pt2.y){return R.e.e1;}
    cl = collinearIntersection(l, R.e.e2);
    if (cl.pt1.x == l.pt1.x && cl.pt1.y == l.pt1.y && cl.pt2.x == l.pt2.x && cl.pt2.y == l.pt2.y){return R.e.e2;}
    cl = collinearIntersection(l, R.e.e3);
    if (cl.pt1.x == l.pt1.x && cl.pt1.y == l.pt1.y && cl.pt2.x == l.pt2.x && cl.pt2.y == l.pt2.y){return R.e.e3;}
    cl = collinearIntersection(l, R.e.e4);
    if (cl.pt1.x == l.pt1.x && cl.pt1.y == l.pt1.y && cl.pt2.x == l.pt2.x && cl.pt2.y == l.pt2.y){return R.e.e4;}*/
    if (isCollinearIntersection(l, R.e.e1)){return R.e.e1;}
    if (isCollinearIntersection(l, R.e.e2)){return R.e.e2;}
    if (isCollinearIntersection(l, R.e.e3)){return R.e.e3;}
    if (isCollinearIntersection(l, R.e.e4)){return R.e.e4;}
    line notFound = {{-1, 0}, {0, 0}};
    return notFound;
}

/* findEdgeWithLineSegment determines (sort of) whether an edge
   exists anywhere in an SPDI containing a particular line
   segment and if it does exist it returns the edge */
line findEdgeWithLineSegment(region s[], int size, line l){
    int i;
    for (i = 0; i < size; ++i){
        line e = hasEdgeWithLineSegment(s[i], l);
        if (e.pt1.x != -1){
            return e;
        }
    }
    line notFound = {{-1, 0}, {0, 0}};
    return notFound;
}

/* hasEdge determines whether a particular region has a particular edge */
bool hasEdge(region R, line e) {
    if (R.e.e1.pt1.x == e.pt1.x && R.e.e1.pt1.y == e.pt1.y && R.e.e1.pt2.x == e.pt2.x && R.e.e1.pt2.y == e.pt2.y){return true;}
    else if (R.e.e2.pt1.x == e.pt1.x && R.e.e2.pt1.y == e.pt1.y && R.e.e2.pt2.x == e.pt2.x && R.e.e2.pt2.y == e.pt2.y){return true;}
    else if (R.e.e3.pt1.x == e.pt1.x && R.e.e3.pt1.y == e.pt1.y && R.e.e3.pt2.x == e.pt2.x && R.e.e3.pt2.y == e.pt2.y){return true;}
    else if (R.e.e4.pt1.x == e.pt1.x && R.e.e4.pt1.y == e.pt1.y && R.e.e4.pt2.x == e.pt2.x && R.e.e4.pt2.y == e.pt2.y){return true;}
    else {return false;}
}

/* hasEntryEdge determines whether a particular region has
   a particular edge as an entry edge to the region */
bool hasEntryEdge(region R, line e) {
    if (!hasEdge(R, e)){return false;}
    point edgeMiddle;
    edgeMiddle.x = (e.pt2.x + e.pt1.x) / 2;
    edgeMiddle.y = (e.pt1.y + e.pt2.y) / 2;
    point regionMiddle;
    double xmax = MAX(max(R.e.e1.pt1.x, R.e.e2.pt1.x, R.e.e3.pt1.x, R.e.e4.pt1.x), max(R.e.e1.pt2.x, R.e.e2.pt2.x, R.e.e3.pt2.x, R.e.e4.pt2.x));
    double xmin = MIN(min(R.e.e1.pt1.x, R.e.e2.pt1.x, R.e.e3.pt1.x, R.e.e4.pt1.x), min(R.e.e1.pt2.x, R.e.e2.pt2.x, R.e.e3.pt2.x, R.e.e4.pt2.x));
    regionMiddle.x = (xmax + xmin) / 2;
    double ymax = MAX(max(R.e.e1.pt1.y, R.e.e2.pt1.y, R.e.e3.pt1.y, R.e.e4.pt1.y), max(R.e.e1.pt2.y, R.e.e2.pt2.y, R.e.e3.pt2.y, R.e.e4.pt2.y));
    double ymin = MIN(min(R.e.e1.pt1.y, R.e.e2.pt1.y, R.e.e3.pt1.y, R.e.e4.pt1.y), min(R.e.e1.pt2.y, R.e.e2.pt2.y, R.e.e3.pt2.y, R.e.e4.pt2.y));
    regionMiddle.y = (ymax + ymin) / 2;
    vector edgeToMiddle;
    edgeToMiddle.x = regionMiddle.x - edgeMiddle.x;
    edgeToMiddle.y = regionMiddle.y - edgeMiddle.y;
    double aScalarProduct = edgeToMiddle.x*R.v.a.x + edgeToMiddle.y*R.v.a.y;
    double bScalarProduct = edgeToMiddle.x*R.v.b.x + edgeToMiddle.y*R.v.b.y;
    double edgeToMiddleMagnitude = sqrt(edgeToMiddle.x*edgeToMiddle.x + edgeToMiddle.y*edgeToMiddle.y);
    double aMagnitude = sqrt(R.v.a.x*R.v.a.x + R.v.a.y*R.v.a.y);
    double bMagnitude = sqrt(R.v.b.x*R.v.b.x + R.v.b.y*R.v.b.y);
    double aVectorAngle = aScalarProduct / (edgeToMiddleMagnitude*aMagnitude);
    double bVectorAngle = bScalarProduct / (edgeToMiddleMagnitude*bMagnitude);
    if (aVectorAngle > 0 || bVectorAngle > 0){return true;}
    return false;
}

/* existsRegionWithEntryEdge determines whether a region exists
   which has a particular edge as an entry edge */
bool existsRegionWithEntryEdge(region s[], line e, int size) {
    int i;
    for (i = 0; i < size; ++i){
        if (hasEntryEdge(s[i], e)){
            return true;
        }
    }
    return false;
}

/* findRegionWithEntryEdge finds a region which has
   a particular entry edge, if such a region exists */
region findRegionWithEntryEdge(region s[], line e, int size) {
    int i;
    for (i = 0; i < size; ++i){
        if (hasEntryEdge(s[i], e)){
            return s[i];
        }
    }
}

/* numberOfReachableEdges finds the number of immediately reachable
   edges of a particular starting interval anywhere in an SPDI */
int numberOfReachableEdges(line I, region s[], int size) {
    line e = findEdgeWithLineSegment(s, size, I);
    if (!existsRegionWithEntryEdge(s, e, size)){
        return 0;
    }
    region R = findRegionWithEntryEdge(s, e, size);
    if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1)
        && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2)
        && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)
        && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
        return 4;
    }
    if ((isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4))){
        return 3;
    }
    if ((isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4))
        || (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4))){
        return 2;
    }
    if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) || isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2)
        || isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) || isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
        return 1;
    }
}

/*struct line * reachableEdges(line I, region s[], int size, line *edge1, line *edge2) {
    int n = numberOfReachableEdges(I, s, size);
    //line edges[n];
    region R = findRegionWithEntryEdge(s, findEdgeWithLineSegment(s, size, I), size);
    if (n == 0){
        //return edges;
    }
    if (n == 1){
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
    }
    if (n == 2){
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
    }
    if (n == 3){
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            edges[2] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            edges[2] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e1);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            edges[2] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            edges[0] = immediateReachability(I, R.v.a, R.v.b, R.e.e2);
            edges[1] = immediateReachability(I, R.v.a, R.v.b, R.e.e3);
            edges[2] = immediateReachability(I, R.v.a, R.v.b, R.e.e4);
            return edges;
        }
    }
}*/

/* hasExitEdge determines whether a particular region has
   a particular edge as an exit edge to the region */
bool hasExitEdge(region R, line e) {
    if (!hasEdge(R, e)){return false;}
    point edgeMiddle;
    edgeMiddle.x = (e.pt2.x + e.pt1.x) / 2;
    edgeMiddle.y = (e.pt1.y + e.pt2.y) / 2;
    point regionMiddle;
    double xmax = MAX(max(R.e.e1.pt1.x, R.e.e2.pt1.x, R.e.e3.pt1.x, R.e.e4.pt1.x), max(R.e.e1.pt2.x, R.e.e2.pt2.x, R.e.e3.pt2.x, R.e.e4.pt2.x));
    double xmin = MIN(min(R.e.e1.pt1.x, R.e.e2.pt1.x, R.e.e3.pt1.x, R.e.e4.pt1.x), min(R.e.e1.pt2.x, R.e.e2.pt2.x, R.e.e3.pt2.x, R.e.e4.pt2.x));
    regionMiddle.x = (xmax + xmin) / 2;
    double ymax = MAX(max(R.e.e1.pt1.y, R.e.e2.pt1.y, R.e.e3.pt1.y, R.e.e4.pt1.y), max(R.e.e1.pt2.y, R.e.e2.pt2.y, R.e.e3.pt2.y, R.e.e4.pt2.y));
    double ymin = MIN(min(R.e.e1.pt1.y, R.e.e2.pt1.y, R.e.e3.pt1.y, R.e.e4.pt1.y), min(R.e.e1.pt2.y, R.e.e2.pt2.y, R.e.e3.pt2.y, R.e.e4.pt2.y));
    regionMiddle.y = (ymax + ymin) / 2;
    vector edgeToMiddle;
    edgeToMiddle.x = regionMiddle.x - edgeMiddle.x;
    edgeToMiddle.y = regionMiddle.y - edgeMiddle.y;
    double aScalarProduct = edgeToMiddle.x*R.v.a.x + edgeToMiddle.y*R.v.a.y;
    double bScalarProduct = edgeToMiddle.x*R.v.b.x + edgeToMiddle.y*R.v.b.y;
    double edgeToMiddleMagnitude = sqrt(edgeToMiddle.x*edgeToMiddle.x + edgeToMiddle.y*edgeToMiddle.y);
    double aMagnitude = sqrt(R.v.a.x*R.v.a.x + R.v.a.y*R.v.a.y);
    double bMagnitude = sqrt(R.v.b.x*R.v.b.x + R.v.b.y*R.v.b.y);
    double aVectorAngle = aScalarProduct / (edgeToMiddleMagnitude*aMagnitude);
    double bVectorAngle = bScalarProduct / (edgeToMiddleMagnitude*bMagnitude);
    if (aVectorAngle < 0 || bVectorAngle < 0){return true;}
    return false;
}

/* existsRegionWithExitEdge determines whether a region exists
   which has a particular edge as an exit edge */
bool existsRegionWithExitEdge(region s[], line e, int size) {
    int i;
    for (i = 0; i < size; ++i){
        if (hasExitEdge(s[i], e)){
            return true;
        }
    }
    return false;
}

/* findRegionWithExitEdge finds a region which has
   a particular exit edge, if such a region exists */
region findRegionWithExitEdge(region s[], line e, int size) {
    int i;
    for (i = 0; i < size; ++i){
        if (hasExitEdge(s[i], e)){
            return s[i];
        }
    }
}

/*bool reach(region s[], int size, line I, line ie, line e) {
    region R = findRegionWithEntryEdge(s, ie, size);
    int numberOfImmediatelyReachableEdges;
    line r;
    if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1)){numberOfImmediatelyReachableEdges++;}
    if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2)){numberOfImmediatelyReachableEdges++;}
    if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)){numberOfImmediatelyReachableEdges++;}
    if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){numberOfImmediatelyReachableEdges++;}
    if (numberOfImmediatelyReachableEdges == 0){return false;}
    if (numberOfImmediatelyReachableEdges == 1){
        line immediatelyReachableEdges[1];
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1)){
            immediatelyReachableEdges[0] = R.e.e1;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2)){
            immediatelyReachableEdges[0] = R.e.e2;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3)){
            immediatelyReachableEdges[0] = R.e.e3;
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e4)){
            immediatelyReachableEdges[0] = R.e.e4;
        }
        if (immediatelyReachableEdges[0].pt1.x == e.pt1.x && immediatelyReachableEdges[0].pt1.y == e.pt1.y && immediatelyReachableEdges[0].pt2.x == e.pt2.x && immediatelyReachableEdges[0].pt2.y == e.pt2.y){
            return true;
        }
        reach(s, size, immediateReachability(I, R.v.a, R.v.b, immediatelyReachableEdges[0]), immediatelyReachableEdges[0], e);
    }
    if (numberOfImmediatelyReachableEdges == 2){
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.a, R.e.e2)){
            line immediatelyReachableEdges[2] = {immediateReachability(I, R.v.a, R.v.b, R.e.e1), immediateReachability(I, R.v.a, R.v.b, R.e.e2)};
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.a, R.e.e3)){
            line immediatelyReachableEdges[2] = {immediateReachability(I, R.v.a, R.v.b, R.e.e1), immediateReachability(I, R.v.a, R.v.b, R.e.e3)};
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e1) && isImmediatelyReachable(I, R.v.a, R.v.a, R.e.e4)){
            line immediatelyReachableEdges[2] = {immediateReachability(I, R.v.a, R.v.b, R.e.e1), immediateReachability(I, R.v.a, R.v.b, R.e.e4)};
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.a, R.e.e3)){
            line immediatelyReachableEdges[2] = {immediateReachability(I, R.v.a, R.v.b, R.e.e2), immediateReachability(I, R.v.a, R.v.b, R.e.e3)};
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e2) && isImmediatelyReachable(I, R.v.a, R.v.a, R.e.e4)){
            line immediatelyReachableEdges[2] = {immediateReachability(I, R.v.a, R.v.b, R.e.e2), immediateReachability(I, R.v.a, R.v.b, R.e.e4)};
        }
        if (isImmediatelyReachable(I, R.v.a, R.v.b, R.e.e3) && isImmediatelyReachable(I, R.v.a, R.v.a, R.e.e4)){
            line immediatelyReachableEdges[2] = {immediateReachability(I, R.v.a, R.v.b, R.e.e3), immediateReachability(I, R.v.a, R.v.b, R.e.e4)};
        }
    }
}*/

/*bool reach(region s[], int size, line I, line e){
    line reachableArray[1];
    reachableArray[0] = I;
    int x;
    for (x = 0; sizeof reachableArray / sizeof reachableArray[0] > x; ++x){
        //struct line * lines[numberOfReachableEdges(reachableArray[x], s, size)];
        struct line * lines = reachableEdges(reachableArray[x], s, size);
        int y;
        for (y=0; sizeof lines / sizeof lines[0] > y; ++y){
            if (findEdgeWithLineSegment(s, size, lines[y]).pt1.x == e.pt1.x && findEdgeWithLineSegment(s, size, lines[y]).pt1.y == e.pt1.y && findEdgeWithLineSegment(s, size, lines[y]).pt2.x == e.pt2.x && findEdgeWithLineSegment(s, size, lines[y]).pt2.y == e.pt2.y) {
                return true;
            }
            reachableArray[sizeof reachableArray / sizeof reachableArray[0]] = lines[y];
        }
    }
    return false;
}*/

/* reach determines whether an edge anywhere in an SPDI
   is reachable by any line segment in the same SPDI */
bool reach(region s[], int size, line I, line e) {
    int i = 0;
    int j = 1;
    line_collection[0] = I;
    region R;
    line eI;
    line ir;
    while (i < j){
        //printf("i = %d, j = %d\n", i, j);
        //printf("I is (%f, %f) to (%f, %f)\n", line_collection[i].pt1.x, line_collection[i].pt1.y, line_collection[i].pt2.x, line_collection[i].pt2.y);
        eI = findEdgeWithLineSegment(s, size, line_collection[i]);
        //printf("eI is (%f, %f) to (%f, %f)\n", eI.pt1.x, eI.pt1.y, eI.pt2.x, eI.pt2.y);
        if (eI.pt1.x == e.pt1.x && eI.pt1.y == e.pt1.y && eI.pt2.x == e.pt2.x && eI.pt2.y == eI.pt2.y){
            return true;
        }
        if (existsRegionWithEntryEdge(s, eI, size)){
            R = findRegionWithEntryEdge(s, eI, size);
            if (isImmediatelyReachable(line_collection[i], R.v.a, R.v.b, R.e.e1)){
                //printf("e1 is (%f, %f) to (%f, %f)\n", R.e.e1.pt1.x, R.e.e1.pt1.y, R.e.e1.pt2.x, R.e.e1.pt2.y);
                ir = immediateReachability(line_collection[i], R.v.a, R.v.b, R.e.e1);
                sort(&ir.pt1.x, &ir.pt1.y, &ir.pt2.x, &ir.pt2.y);
                //printf("ir is (%f, %f) to (%f, %f)\n", ir.pt1.x, ir.pt1.y, ir.pt2.x, ir.pt2.y);
                line_collection[j] = ir;
                ++j;
            }
            if (isImmediatelyReachable(line_collection[i], R.v.a, R.v.b, R.e.e2)){
                //printf("e2 is (%f, %f) to (%f, %f)\n", R.e.e2.pt1.x, R.e.e2.pt1.y, R.e.e2.pt2.x, R.e.e2.pt2.y);
                ir = immediateReachability(line_collection[i], R.v.a, R.v.b, R.e.e2);
                sort(&ir.pt1.x, &ir.pt1.y, &ir.pt2.x, &ir.pt2.y);
                //printf("ir is (%f, %f) to (%f, %f)\n", ir.pt1.x, ir.pt1.y, ir.pt2.x, ir.pt2.y);
                line_collection[j] = ir;
                ++j;
            }
            if (isImmediatelyReachable(line_collection[i], R.v.a, R.v.b, R.e.e3)){
                //printf("e3 is (%f, %f) to (%f, %f)\n", R.e.e3.pt1.x, R.e.e3.pt1.y, R.e.e3.pt2.x, R.e.e3.pt2.y);
                ir = immediateReachability(line_collection[i], R.v.a, R.v.b, R.e.e3);
                sort(&ir.pt1.x, &ir.pt1.y, &ir.pt2.x, &ir.pt2.y);
                //printf("ir is (%f, %f) to (%f, %f)\n", ir.pt1.x, ir.pt1.y, ir.pt2.x, ir.pt2.y);
                line_collection[j] = ir;
                ++j;
            }
            if (isImmediatelyReachable(line_collection[i], R.v.a, R.v.b, R.e.e4)){
                //printf("e4 is (%f, %f) to (%f, %f)\n", R.e.e4.pt1.x, R.e.e4.pt1.y, R.e.e4.pt2.x, R.e.e4.pt2.y);
                ir = immediateReachability(line_collection[i], R.v.a, R.v.b, R.e.e4);
                sort(&ir.pt1.x, &ir.pt1.y, &ir.pt2.x, &ir.pt2.y);
                //printf("ir is (%f, %f) to (%f, %f)\n", ir.pt1.x, ir.pt1.y, ir.pt2.x, ir.pt2.y);
                line_collection[j] = ir;
                ++j;
            }
        }
        ++i;
    }
    return false;
}

/*point middlePoint(point p1, point p2, point p3, point p4){
    point p;
    p.x = max(p1.x, p2.x, p3.x, p4.x) + min(p1.x, p2.x, p3.x, p4.x) / 2;
    p.y = max(p1.y, p2.y, p3.y, p4.y) + min(p1.y, p2.y, p3.y, p4.y) / 2;
    return p;
}*/

/* succ returns an array of lines representing the reachable line segments of
   successor edges of a given edge list starting from a given starting interval */
struct line * succ(line succ[], line edgeList[], int numberOfEdges, line I, region s[], int size){
    succ[numberOfEdges + 1];
    succ[0] = I;
    int i = 0;
    region R;
    line l;
    while (i <= numberOfEdges - 1){
        R = findRegionWithEntryEdge(s, edgeList[i], size);
        l = immediateReachability(I, R.v.a, R.v.b, edgeList[(i+1)%numberOfEdges]);
        succ[i+1] = l;
        I = l;
        i = (i + 1) % numberOfEdges;
        //printf("succ[%d] is (%f, %f) to (%f, %f)\n", i, succ[i].pt1.x, succ[i].pt1.y, succ[i].pt2.x, succ[i].pt2.y);
    }
    return succ;
}

/* controllabilityKernel returns an array of lines representing all the line segments
   of a controllability kernel given an edge list starting from a starting interval.
   At the moment it calculates the successor and predecessor line segments itself
   rather than making use of other functions specifically made to do the job */
struct line * controllabilityKernel(line edgeList[], int numberOfEdges, line I, region s[], int size){
    int i = 0;
    int j;
    region R;
    line l;
    line l1;
    line l2;
    point p;
    line succ[numberOfEdges + 1];
    line pre[numberOfEdges + 1];
    //line kernel[numberOfEdges];
    line* kernel = malloc(sizeof(line) * numberOfEdges);
    //point innerKernel[numberOfEdges];
    //point outerKernel[numberOfEdges];
    succ[i] = I;
    pre[i] = I;
    succ[0] = I;
    pre[0] = I;
    i = 1;
    // findRegionWithEntryEdge()
    while (i <= numberOfEdges){
        for (j = 0; j < size; ++j){
            if (hasEntryEdge(s[j], edgeList[i-1])){
                R = s[j];
                break;
            }
        }
        l = immediateReachability(succ[i-1], R.v.a, R.v.b, edgeList[i%numberOfEdges]);
        //printf("edgeList[%d] is (%f, %f) to (%f, %f)\n", i, edgeList[i%numberOfEdges].pt1.x, edgeList[i%numberOfEdges].pt1.y, edgeList[i%numberOfEdges].pt2.x, edgeList[i%numberOfEdges].pt2.y);
        succ[i] = l;
        //printf("succ[%d] is (%f, %f) to (%f, %f)\n", i, succ[i].pt1.x, succ[i].pt1.y, succ[i].pt2.x, succ[i].pt2.y);
        //I = l;
        ++i;
    }
    i = 1;
    //findRegionWithExitEdge()
    while (i <= numberOfEdges){
        for (j = 0; j < size; ++j){
            if (hasExitEdge(s[j], edgeList[i-1])){
                R = s[j];
                break;
            }
        }
        l = immediateReachabilityReverse(pre[i-1], R.v.a, R.v.b, edgeList[(numberOfEdges - i)%numberOfEdges], edgeList[(numberOfEdges - i + 1)%numberOfEdges]);
        pre[i] = l;
        //printf("pre[%d] is (%f, %f) to (%f, %f)\n", i, pre[i].pt1.x, pre[i].pt1.y, pre[i].pt2.x, pre[i].pt2.y);
        //I = l;
        ++i;
    }
    kernel[0] = collinearIntersection(succ[0], collinearIntersection(succ[numberOfEdges], pre[numberOfEdges]));
    i = 1;
    while (i < numberOfEdges){
        kernel[i] = collinearIntersection(succ[i], pre[numberOfEdges - i]);
        ++i;
    }
    /*for (i = 0; i <= numberOfEdges; ++i){
        l = succ[i];
        printf("succ[%d] = (%f, %f) to (%f, %f)\n", i, l.pt1.x, l.pt1.y, l.pt2.x, l.pt2.y);
    }
    for (i = 0; i <= numberOfEdges; ++i){
        l = pre[i];
        printf("pre[%d] = (%f, %f) to (%f, %f)\n", i, l.pt1.x, l.pt1.y, l.pt2.x, l.pt2.y);
    }
    for (i = 0; i < numberOfEdges; ++i){
        l = kernel[i];
        printf("kernel[%d] = (%f, %f) to (%f, %f)\n", i, l.pt1.x, l.pt1.y, l.pt2.x, l.pt2.y);
    }*/
    return kernel;
}

/* innerKernel is meant to return an array of points representing the points
   of the inner part of a kernel which can be joined together with straight
   lines to find the inner kernel. At the moment it just prints both the inner
   and outer points of the kernel because both are worked out in this function
   and having another function for finding the outer kernel would be recomputing
   the same results, as both the inner and outer paoints of the kernel need
   to be compared in order for each of them to be identified */
struct point * innerKernel(line kernel[], int numberOfEdges){
//void innerKernel(line kernel[], int numberOfEdges){
    int i = 0;
    line l1;
    line l2;
    point array1[numberOfEdges];
    point array2[numberOfEdges];
    point temp;
    for (i = 0; i < numberOfEdges; ++i){
        printf("i is %d\n", i);
        if (i == 0){l1.pt1 = kernel[i].pt1;}
        else {l1.pt1 = l1.pt2;}
        l1.pt2 = kernel[(i+1)%numberOfEdges].pt1;
        printf("l1 = (%f, %f) to (%f, %f)\n", l1.pt1.x, l1.pt1.y, l1.pt2.x, l1.pt2.y);
        if (i == 0){l2.pt1 = kernel[i].pt2;}
        else {l2.pt1 = l2.pt2;}
        l2.pt2 = kernel[(i+1)%numberOfEdges].pt2;
        printf("l2 = (%f, %f) to (%f, %f)\n", l2.pt1.x, l2.pt1.y, l2.pt2.x, l2.pt2.y);
        if ((l1.pt1.x == l1.pt2.x && l1.pt1.y == l1.pt2.y) || (l2.pt1.x == l2.pt2.x && l2.pt1.y == l2.pt2.y)){
            printf("1yes\n");
            array1[i] = l1.pt1;
            array1[i+1] = l1.pt2;
            array2[i] = l2.pt1;
            array2[i+1] = l2.pt2;
        }
        if ((l1.pt1.x == l2.pt2.x && l1.pt1.y == l2.pt2.y) || (l2.pt1.x == l2.pt2.x && l1.pt1.y == l1.pt2.y)){
            printf("2yes\n");
            array1[i] = l1.pt1;
            array1[i+1] = l2.pt2;
            array2[i] = l2.pt1;
            array2[i+1] = l1.pt2;
        }
        if ((l1.pt2.y - l1.pt1.y) / (l1.pt2.x - l1.pt1.x) == (l2.pt2.y - l2.pt1.y) / (l2.pt2.x - l2.pt1.x)){
            printf("lines are parallel\n");
            array1[i] = l1.pt1;
            array1[i+1] = l1.pt2;
            array2[i] = l2.pt1;
            array2[i+1];
        }
        else {
            printf("lines are not parallel\n");
            point ip = lineIntersection(l1, l2);
            printf("ip is (%f, %f)\n", ip.x, ip.y);
            point mp = middlePoint(l1.pt1, l1.pt2, l2.pt1, l2.pt2);
            //printf("mp is (%f, %f)\n", mp.x, mp.y);
            /*if (((ip.x > l1.pt1.x && ip.x > l2.pt1.x && ip.x < l1.pt2.x && ip.x < l2.pt2.x)
            || (ip.x < l1.pt1.x && ip.x < l2.pt1.x && ip.x > l1.pt2.x && ip.x > l2.pt2.x))
            && ((ip.y > l1.pt1.y && ip.y > l2.pt1.y && ip.y < l1.pt2.y && ip.y < l2.pt2.y)
            || (ip.y < l1.pt1.y && ip.y < l2.pt1.y && ip.y > l1.pt2.y && ip.y > l2.pt2.y))){*/
            if (ip.x > min(l1.pt1.x, l1.pt2.x, l2.pt1.x, l2.pt2.x)
             && ip.x < max(l1.pt1.x, l1.pt2.x, l2.pt1.x, l2.pt2.x)
             && ip.y > min(l1.pt1.y, l1.pt2.y, l2.pt1.y, l2.pt2.y)
             && ip.y < max(l1.pt1.y, l1.pt2.y, l2.pt1.y, l2.pt2.y)){
                printf("lines cross\n");
                printf("before l1 is (%f, %f) to (%f, %f)\n", l1.pt1.x, l1.pt1.y, l1.pt2.x, l1.pt2.y);
                printf("before l2 is (%f, %f) to (%f, %f)\n", l2.pt1.x, l2.pt1.y, l2.pt2.x, l2.pt2.y);
                temp = l1.pt2;
                l1.pt2 = l2.pt2;
                l2.pt2 = temp;
                printf("after l1 is (%f, %f) to (%f, %f)\n", l1.pt1.x, l1.pt1.y, l1.pt2.x, l1.pt2.y);
                printf("after l2 is (%f, %f) to (%f, %f)\n", l2.pt1.x, l2.pt1.y, l2.pt2.x, l2.pt2.y);
                array1[i] = l1.pt1;
                array1[i+1] = l1.pt2;
                array2[i] = l2.pt1;
                array2[i+1] = l2.pt2;
            }
            else {
                printf("lines do not cross\n");
            }
        }
    }
    double length1 = 0;
    double length2 = 0;
    for (i = 0; i < numberOfEdges - 1; ++i){
        length1 = length1 + distance(array1[i], array1[i+1]);
        length2 = length2 + distance(array2[i], array2[i+1]);
    }
    printf("length1 is %f\n", length1);
    printf("length2 is %f\n", length2);
    if (length1 < length2){
        printf("inner points are:\n");
        for (i = 0; i < numberOfEdges; ++i){
            printf("(%f, %f)\n", array1[i].x, array1[i].y);
        }
        printf("outer points are:\n");
        for (i = 0; i < numberOfEdges; ++i){
            printf("array2[%d] is (%f, %f)\n", i, array2[i].x, array2[i].y);
        }
        //return array1;
    }
    if (length1 > length2){
        printf("inner points are:\n");
        for (i = 0; i < numberOfEdges; ++i){
            printf("(%f, %f)\n", array2[i].x, array2[i].y);
        }
        printf("outer points are:\n");
        for (i = 0; i < numberOfEdges; ++i){
            printf("array2[%d] is (%f, %f)\n", i, array1[i].x, array1[i].y);
        }
        //return array2;
    }
}

/* isInnerKernel determines whether a calculated inner kernel actually is
   an inner part of a kernel as a given kernel is not guaranteed to have an
   inner kernel although all kernels are guaranteed to have outer kernels */
bool isInnerKernel(point innerKernel[], int numberOfEdges){
    int i = 0;
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double length = 0;
    for (i = 0; i < numberOfEdges - 1; ++i){
        length = length + distance(innerKernel[i], innerKernel[i+1]);
        xmin = MIN(innerKernel[i].x, innerKernel[i+1].x);
        xmax = MAX(innerKernel[i].x, innerKernel[i+1].x);
        ymin = MIN(innerKernel[i].y, innerKernel[i+1].y);
        ymax = MAX(innerKernel[i].y, innerKernel[i+1].y);
    }
    if (xmax - xmin > 0 && ymax - ymin > 0){return true;}
    else {return false;}
}

/* findLineIndexInArray returns the index number of a given line
   in an array of lines and returns -1 if it is not found */
int findLineIndexInArray(line array[], int num_elements, line element){
    int i;
    for (i=0; i < num_elements; ++i){
        if (array[i].pt1.x == element.pt1.x && array[i].pt1.y == element.pt1.y && array[i].pt2.x == element.pt2.x && array[i].pt2.y == element.pt2.y){
            return i;
        }
    }
    return -1;
}

/* escape returns a particular line segment of an invariance kernel
   with the line segment being the limits of a particular edge where
   if a trajectory were to be within that line segment then it will
   be within the invariance kernel and therefore will never leave that
   set. I find this hard to test */
line escape(line e, line I, line edgeList[], int numberOfEdges, region s[], int size){
    int numberOfPoints = 0;
    int i = 0;
    region R;
    line l;
    line x1;
    line x2;
    line x3;
    //printf("number of edges = %d\n", numberOfEdges);
    //line e1 = findEdgeWithLineSegment(edgeList, numberOfEdges, I);
    line e1 = findEdgeWithLineSegment(s, size, I);
    if (e1.pt1.x == e.pt1.x && e1.pt1.y == e.pt1.y && e1.pt2.x == e.pt2.x && e1.pt2.y == e.pt2.y){
        x1 = I;
        ++numberOfPoints;
    }
    //printf("e1 is (%f, %f) to (%f, %f)\n", e1.pt1.x, e1.pt1.y, e1.pt2.x, e1.pt2.y);
    i = findLineIndexInArray(edgeList, numberOfEdges, e1);
    while (numberOfPoints < 3){
        //printf("number of points is %d\n", numberOfPoints);
        //printf("i = %d\n", i);
        //printf("line I is (%f, %f) to (%f, %f)\n", I.pt1.x, I.pt1.y, I.pt2.x, I.pt2.y);
        //printf("edge[%d] is (%f, %f) to (%f, %f)\n", i, edgeList[i].pt1.x, edgeList[i].pt1.y, edgeList[i].pt2.x, edgeList[i].pt2.y);
        R = findRegionWithEntryEdge(s, edgeList[i], size);
        l = immediateReachability(I, R.v.a, R.v.b, edgeList[(i+1)%numberOfEdges]);
        //printf("l = (%f, %f) to (%f, %f)\n", l.pt1.x, l.pt1.y, l.pt2.x, l.pt2.y);
        //e1 = findEdgeWithLineSegment(edgeList, numberOfEdges, l);
        e1 = findEdgeWithLineSegment(s, size, l);
        //printf("e1 is (%f, %f) to (%f, %f)\n", e1.pt1.x, e1.pt1.y, e1.pt2.x, e1.pt2.y);
        if (e1.pt1.x == e.pt1.x && e1.pt1.y == e.pt1.y && e1.pt2.x == e.pt2.x && e1.pt2.y == e.pt2.y){
            if (numberOfPoints == 0){
                x1 = l;
                ++numberOfPoints;
            }
            else if (numberOfPoints == 1){
                x2 = l;
                ++numberOfPoints;
            }
            else if (numberOfPoints == 2){
                x3 = l;
                ++numberOfPoints;
            }
        }
        I = l;
        i = (i + 1) % numberOfEdges;
        //printf("number of points is %d\n", numberOfPoints);
    }
    printf("x1 is (%f, %f) to (%f, %f)\n", x1.pt1.x, x1.pt1.y, x1.pt2.x, x1.pt2.y);
    printf("x2 is (%f, %f) to (%f, %f)\n", x2.pt1.x, x2.pt1.y, x2.pt2.x, x2.pt2.y);
    printf("x3 is (%f, %f) to (%f, %f)\n", x3.pt1.x, x3.pt1.y, x3.pt2.x, x3.pt2.y);
    sort(&x1.pt1.x, &x1.pt1.y, &x1.pt2.x, &x1.pt2.y);
    sort(&x2.pt1.x, &x2.pt1.y, &x2.pt2.x, &x2.pt2.y);
    sort(&x3.pt1.x, &x3.pt1.y, &x3.pt2.x, &x3.pt2.y);
    double u1 = distance(x1.pt1, x2.pt1);
    double u2 = distance(x2.pt1, x3.pt1);
    double d1 = distance(x1.pt2, x2.pt2);
    double d2 = distance(x2.pt2, x3.pt2);
    double ur;
    if (u2 == 0){ur = 0;}
    else {ur = u2 / u1;}
    double dr;
    if (d2 == 0){dr = 0;}
    else {dr = d2 / d1;}
    line x;
    if (ur < 1){
        x.pt1.x = x1.pt1.x + (u1 / (1 - ur));
        x.pt1.y = x1.pt1.y + (u1 / (1 - ur));
    }
    else {
        x.pt1.x = e.pt1.x;
        x.pt1.y = e.pt1.y;
    }
    if (dr < 1){
        x.pt2.x = x1.pt2.x + (d1 / (1 - dr));
        x.pt2.y = x1.pt2.y + (d1 / (1 - dr));
    }
    else {
        x.pt2.x = e.pt2.x;
        x.pt2.y = e.pt2.y;
    }
    printf("u1 is %f, u2 is %f\n", u1, u2);
    printf("ur is %f, dr is %f\n", ur, dr);
    printf("x = (%f, %f) to (%f, %f)\n", x.pt1.x, x.pt1.y, x.pt2.x, x.pt2.y);
    return x;
}

int main()
{
    struct region R1;
    struct region R2;
    struct region R3;
    struct region R4;

    regionEdges re1 = { {{1, 1}, {2, 1}}, {{1, 1}, {1, 2}}, {{1, 2}, {2, 2}}, {{2, 1}, {2, 2}} };
    regionVectors rv1 = { {1, -1}, {1, -1} };
    regionEdges re2 = { {{2,1},{3,1}}, {{2,1},{2,2}}, {{2,2},{3,2}}, {{3,1},{3,2}} };
    regionVectors rv2 = { {1, 1}, {1, 1} };
    regionEdges re3 = { {{2,2},{3,2}}, {{2,2},{2,3}}, {{2,3},{3,3}}, {{3,2},{3,3}} };
    regionVectors rv3 = { {-1, 1}, {-1, 1.5} };
    regionEdges re4 = { {{1,2},{2,2}}, {{1,2},{1,3}}, {{1,3},{2,3}}, {{2,2},{2,3}} };
    regionVectors rv4 = { {-1, -1}, {-1, -1} };
    R1.e = re1;
    R1.v = rv1;
    R2.e = re2;
    R2.v = rv2;
    R3.e = re3;
    R3.v = rv3;
    R4.e = re4;
    R4.v = rv4;

    //sort(&I.pt1.x, &I.pt2.x, &I.pt1.y, &I.pt2.y);
    //sort(&e.pt1.x, &e.pt2.x, &e.pt1.y, &e.pt2.y);

    region SPDI1[4] = {R1, R2, R3, R4};
    region SPDI2[2] = {R1, R2};

    line initial = { {2, 1.25}, {2, 1.75} };
    line e1 = { {2, 1}, {2, 2} };
    line e2 = { {2, 2}, {3, 2} };
    line e3 = { {2, 2}, {2, 3} };
    line e4 = { {1, 2}, {2, 2} };
    line edgeList[4] = {e1, e2, e3, e4};

    int size = sizeof(SPDI1) / sizeof(SPDI1[0]);

    //line* s = malloc(sizeof(line) * 5);
    //s = succ(s, edgeList, 4, initial, SPDI, size);

    //escape(e1, initial, edgeList, 4, SPDI, size);

    //line* k = malloc(sizeof(line) * 4);
    line* k;
    k = controllabilityKernel(edgeList, 4, initial, SPDI1, size);
    int i;
    for (i = 0; i < 4; ++i) {
        printf("k[%d] is (%f, %f) to (%f, %f)\n", i, k[i].pt1.x, k[i].pt1.y, k[i].pt2.x, k[i].pt2.y);
    }
    //escape(e3, initial, edgeList, 4, SPDI1, size);
    //point* inner = malloc(sizeof(point) * 4);
    point* inner;
    inner = innerKernel(k, 4);

    line test1 = {{1, 1.25}, {1, 1.75}};
    line test2 = {{1, 1}, {2, 1}};
    //if (isImmediatelyReachable(test1, R1.v.a, R1.v.b, test2)){printf("a\n");}
    if (reach(SPDI1, size, test1, test2)){printf("yes\n");}else {printf("no\n");}

    /*struct point * innerK = innerKernel(k, 4);
    printf("inner kernel:\n");*/
    /*int i = 0;
    for (i = 0; i < (sizeof(innerK) / sizeof(innerK[0])); ++i){
        printf("(%f, %f)\n", innerK[i].x, innerK[i].y);
    }*/

    //free(s);
    free(k);
    free(inner);
    return 0;
}
