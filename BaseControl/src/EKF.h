#pragma once

struct EKF3 {
    float x[3];     // [x, y, theta]
    float P[3][3];  // covariance
  
    float Q[3][3];  // process noise
    float R[3][3];  // measurement noise
  
    float b;        // wheelbase (m)
  
    static float wrapPi(float a) {
      while (a >  3.14159265f) a -= 2.0f*3.14159265f;
      while (a < -3.14159265f) a += 2.0f*3.14159265f;
      return a;
    }
  
    // --- 3x3 helpers (add, mult, transpose mult, inverse) ---
    // Keep these small + inline for speed.
  
    static void matAdd33(const float A[3][3], const float B[3][3], float C[3][3]) {
      for (int i=0;i<3;i++) for (int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j];
    }
  
    static void matMul33(const float A[3][3], const float B[3][3], float C[3][3]) {
      for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        float s=0;
        for (int k=0;k<3;k++) s += A[i][k]*B[k][j];
        C[i][j]=s;
      }
    }
  
    static void matMul33T(const float A[3][3], const float B[3][3], float C[3][3]) {
      // C = A * B^T
      for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        float s=0;
        for (int k=0;k<3;k++) s += A[i][k]*B[j][k];
        C[i][j]=s;
      }
    }
  
    static bool inv33(const float M[3][3], float invOut[3][3]) {
      // Compute inverse via adjugate / determinant (OK for 3x3).
      float a=M[0][0], b=M[0][1], c=M[0][2];
      float d=M[1][0], e=M[1][1], f=M[1][2];
      float g=M[2][0], h=M[2][1], i=M[2][2];
  
      float A =  (e*i - f*h);
      float B = -(d*i - f*g);
      float C =  (d*h - e*g);
      float D = -(b*i - c*h);
      float E =  (a*i - c*g);
      float F = -(a*h - b*g);
      float G =  (b*f - c*e);
      float H = -(a*f - c*d);
      float I =  (a*e - b*d);
  
      float det = a*A + b*B + c*C;
      if (fabs(det) < 1e-9f) return false;
      float invDet = 1.0f / det;
  
      invOut[0][0]=A*invDet; invOut[0][1]=D*invDet; invOut[0][2]=G*invDet;
      invOut[1][0]=B*invDet; invOut[1][1]=E*invDet; invOut[1][2]=H*invDet;
      invOut[2][0]=C*invDet; invOut[2][1]=F*invDet; invOut[2][2]=I*invDet;
      return true;
    }
  
    // Omniwheel predict: dx/dy/dtheta already in world frame (F = I => P += Q)
    void predictOmni(float dx, float dy, float dtheta) {
      x[0] += dx;
      x[1] += dy;
      x[2]  = wrapPi(x[2] + dtheta);
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          P[i][j] += Q[i][j];
    }
  
    void predict(float dL, float dR) {
      float ds = 0.5f*(dR + dL);
      float dth = (dR - dL)/b;
      float phi = x[2] + 0.5f*dth;
  
      // state
      x[0] += ds * cosf(phi);
      x[1] += ds * sinf(phi);
      x[2]  = wrapPi(x[2] + dth);
  
      // F
      float Fm[3][3] = {
        {1, 0, -ds*sinf(phi)},
        {0, 1,  ds*cosf(phi)},
        {0, 0,  1}
      };
  
      // P = F P F^T + Q
      float FP[3][3], FPFt[3][3], PFt[3][3];
      matMul33(Fm, P, FP);
      matMul33T(FP, Fm, FPFt);   // (F P) * F^T
      matAdd33(FPFt, Q, P);
    }
  
    bool updateFiducial(float xf, float yf, float thf) {
      // innovation y = z - x, angle wrapped
      float yv[3] = {
        xf - x[0],
        yf - x[1],
        wrapPi(thf - x[2])
      };
  
      // Optional: simple outlier gate (tune thresholds)
      if (fabs(yv[0]) > 1.0f || fabs(yv[1]) > 1.0f) { // e.g. >1m jump
        return false; // ignore this measurement
      }
  
      // S = P + R
      float S[3][3];
      matAdd33(P, R, S);
  
      // invS
      float invS[3][3];
      if (!inv33(S, invS)) return false;
  
      // K = P * invS
      float K[3][3];
      matMul33(P, invS, K);
  
      // x = x + K*y
      float dx0 = K[0][0]*yv[0] + K[0][1]*yv[1] + K[0][2]*yv[2];
      float dx1 = K[1][0]*yv[0] + K[1][1]*yv[1] + K[1][2]*yv[2];
      float dx2 = K[2][0]*yv[0] + K[2][1]*yv[1] + K[2][2]*yv[2];
  
      x[0] += dx0;
      x[1] += dx1;
      x[2]  = wrapPi(x[2] + dx2);
  
      // P = (I - K) P   (since H=I)
      float IminusK[3][3] = {
        {1-K[0][0],   -K[0][1],   -K[0][2]},
        {  -K[1][0], 1-K[1][1],   -K[1][2]},
        {  -K[2][0],   -K[2][1], 1-K[2][2]}
      };
      float newP[3][3];
      matMul33(IminusK, P, newP);
      for (int r=0;r<3;r++) for (int c=0;c<3;c++) P[r][c]=newP[r][c];
  
      return true;
    }
  };
  