#include<math.h>
#include<stdio.h>
// Mechanical leg segments measured from axis to axis
const float L1 = 0.47;
const float L2 = 1.7;
const float L3 = 3.2;
// Mechanical joint limits
const float S_LIM[] = {-90, 90};
const float E_LIM[] = {-90, 90};
const float W_LIM[] = {-90, 90};
// 3D point in real space
struct point3D {
    float x;
    float y;
    float z;
};
// Saves servo values and relevant leg values
struct leg{
    const struct point3D offset;
    const int servoS;
    const int servoE;
    const int servoW;
    struct point3D currentPos;
    float angleS;
    float angleE;
    float angleW;
};
// Performs inverse kinematics and copies result to leg
int writeLeg(struct leg *leg, struct point3D point) {
    float dx = point.x - leg->offset.x;
    float dy = point.y - leg->offset.y;
    float dz = point.z - leg->offset.z;
    float s = atan2f(dy, dx);
    float fx = dx - L1 * cosf(s);
    float fy = dy - L1 * sinf(s);
    float f = sqrtf(fx * fx + fy * fy + dz * dz);
    float e = asinf(dz / f) + acosf((powf(L2, 2) - powf(L3, 2) + powf(f, 2)) / (2 * L2 * f));
    float w = -1 * acosf((powf(L2, 2) - powf(L3, 2) + powf(f, 2)) / (2 * L2 * f)) - acosf((powf(L3, 2) - powf(L2, 2) + powf(f, 2)) / (2 * L3 * f));
    leg->angleS = s * 180 / M_PI;
    leg->angleE = e * 180 / M_PI;
    leg->angleW = w * 180 / M_PI;
    leg->currentPos = point;
    return 0;
}
// Moves leg to target point
int moveLeg(struct leg *leg, struct point3D point) {
    if (writeLeg(leg, point) != 0) {
        perror("Error in writeLeg: ");
        return 1;
    }

    // Boundary check the leg angles
    if(leg->angleS < S_LIM[0] || leg->angleS > S_LIM[1]) return 1; 
    if(leg->angleE < E_LIM[0] || leg->angleE > E_LIM[1]) return 1; 
    if(leg->angleW < W_LIM[0] || leg->angleW > W_LIM[1]) return 1; 

    // Use I2C to move servos here

    return 0;
}
int main() {
    // Initialize legs with offset within {}
    // Do not touch servo indices
    struct leg leg_1 = {{0,0,0},0,1,2};
    struct leg leg_2 = {{0,0,0},3,4,5};
    struct leg leg_3 = {{0,0,0},6,7,8};
    struct leg leg_4 = {{0,0,0},9,10,11};
    struct leg leg_5 = {{0,0,0},12,13,14};
    struct leg leg_6 = {{0,0,0},15,16,17};

    // Home positions for each leg
    struct point3D h1 = {1,1,1};
    struct point3D h2 = {1,1,1};
    struct point3D h3 = {1,1,1};
    struct point3D h4 = {1,1,1};
    struct point3D h5 = {1,1,1};
    struct point3D h6 = {1,1,1};

    // Move legs to initial position
    moveLeg(&leg_1, h1);
    moveLeg(&leg_2, h2);
    moveLeg(&leg_3, h3);
    moveLeg(&leg_4, h4);
    moveLeg(&leg_5, h5);
    moveLeg(&leg_6, h6);

    // Initialize communication loop

    // Build stride pattern
    return 0;
}