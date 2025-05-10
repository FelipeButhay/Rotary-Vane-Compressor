#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <iostream>

#define N_PARTICLES_CONST 0
#define N_PALLETES 6
#define sx 1600
#define sy 900
#define speed 100
#define radius 5
#define mass 1
#define ANG_V 0.6f
#define V_MAX 300
#define T_FPS 60.0f
#define SUBSTEPS 3

float RanNorm() {
    return GetRandomValue(0, 10000)/10000.0f;
}

float RanSign() {
    return GetRandomValue(0,1)*2 - 1;
}

float mod(float a, float b) {
    return std::fmod(std::fmod(a, b) + b, b);
}

struct Particle {
    Vector2 p, v;
    float r, m;
    Color c;
};

struct Pallete {
    Vector2 p1, p2;
};

bool IsTouching(Particle& p1, Particle& p2) {
    return Vector2Distance(p1.p, p2.p) < p1.r + p2.r;
}

Vector2 ElasticCollision(Particle& p1, Particle& p2){
    Vector2 DeltaPos = Vector2Subtract(p1.p, p2.p);
    Vector2 DeltaVel = Vector2Subtract(p1.v, p2.v);

    float MassFactor = 2*p2.m / (p1.m + p2.m);

    float sqrDistance = Vector2Length(DeltaPos) * Vector2Length(DeltaPos);
    float VelocityFactor = Vector2DotProduct(DeltaVel, DeltaPos) / sqrDistance;

    return Vector2Subtract(p1.v, Vector2Scale(DeltaPos, MassFactor * VelocityFactor));
}

Vector2 ReflectVelocity(Vector2 velocity, float m) {
    Vector2 normal = Vector2Normalize((Vector2){-m, 1}); // vector normal unitario
    float dotProd = Vector2DotProduct(velocity, normal);
    return Vector2Subtract(velocity, Vector2Scale(normal, 2 * dotProd));
}

Vector2 StaticCollisionCircle(Vector2 c, Particle& pcl){
    Vector2 delta = Vector2Subtract(c, pcl.p);
    float m = delta.y / delta.x;
    return ReflectVelocity(pcl.v, -1/m);
}

bool IsCollidingWithThickLine(Particle& pcl, Vector2 p1, Vector2 p2, float thickness, Vector2* normal_out = nullptr) {
    Vector2 seg = Vector2Subtract(p2, p1);     // Vector del segmento
    Vector2 pt = Vector2Subtract(pcl.p, p1);       // Vector desde p1 al punto de la partícula

    float seg_len_sq = Vector2LengthSqr(seg);  // Largo al cuadrado del segmento
    float t = Vector2DotProduct(pt, seg) / seg_len_sq;

    // Clampear t entre 0 y 1 para que no se pase de los extremos
    t = fmaxf(0.0f, fminf(1.0f, t));

    // Punto más cercano sobre el segmento
    Vector2 closest = Vector2Add(p1, Vector2Scale(seg, t));

    // Vector desde el punto más cercano hasta el centro de la partícula
    Vector2 to_circle = Vector2Subtract(pcl.p, closest);
    float dist_sq = Vector2LengthSqr(to_circle);
    float min_dist = pcl.r + thickness / 2.0f;

    if (dist_sq < min_dist * min_dist) {
        if (normal_out != nullptr) {
            float dist = sqrtf(dist_sq);
            if (dist > 0.0001f)
                *normal_out = Vector2Scale(to_circle, 1.0f / dist);
            else
                *normal_out = Vector2Normalize({-seg.y, seg.x}); // arbitraria
        }
        return true;
    }

    return false;
}

bool HandleRectCollision(Particle& pcl, Rectangle rect) {
    // Encuentra el punto más cercano del rectángulo al centro de la partícula
    float closestX = fmaxf(rect.x, fminf(pcl.p.x, rect.x + rect.width));
    float closestY = fmaxf(rect.y, fminf(pcl.p.y, rect.y + rect.height));

    // Calcula distancia desde ese punto hasta el centro de la partícula
    Vector2 closest = { closestX, closestY };
    Vector2 to_circle = Vector2Subtract(pcl.p, closest);
    float dist_sq = Vector2LengthSqr(to_circle);
    float min_dist = pcl.r;

    if (dist_sq < min_dist * min_dist) {
        float dist = sqrtf(dist_sq);
        Vector2 normal;

        if (dist > 0.0001f) {
            normal = Vector2Scale(to_circle, 1.0f / dist);
        } else {
            // Si la partícula está justo en la esquina del rectángulo, elegimos una normal arbitraria
            normal = {1.0f, 0.0f};
        }

        // Corrige posición
        float penetration = min_dist - dist;
        pcl.p = Vector2Add(pcl.p, Vector2Scale(normal, penetration));

        // Refleja la velocidad
        float vn = Vector2DotProduct(pcl.v, normal);
        pcl.v = Vector2Subtract(pcl.v, Vector2Scale(normal, 2 * vn));

        return true;
    }

    return false;
}

bool IsPointInRect(Vector2 point, Rectangle rect) {
    return (
        point.x >= rect.x &&
        point.x <= rect.x + rect.width &&
        point.y >= rect.y &&
        point.y <= rect.y + rect.height
    );
}

int main() {
    SetRandomSeed(GetTime()*8134567);

    float palleteThickness = sy*0.05;
    
    Vector2 Stator = {sx*0.7f, sy*0.5f};
    float GreatRadiusOut = sy*0.43f;
    float GreatRadius = sy*0.38f;
    
    Vector2 Rotor = {sx*0.7f, sy*0.58f};
    float RotorRadius = sy*0.25f;

    float inTakeUp = sy*0.2f;
    float inTakeDw = sy*0.5f;

    float outTakeUp = sy*0.81f;
    float outTakeDw = sy*0.88f;

    float inTakePipeA1 = 1*PI + asin(fabs(inTakeUp - Stator.y) / GreatRadius);
    float inTakePipeA2 = 1*PI + asin(fabs(inTakeDw - Stator.y) / GreatRadius);

    float outTakePipeA1 = PI - asin((outTakeUp - Stator.y) / GreatRadius);
    float outTakePipeA2 = PI - asin((outTakeDw - Stator.y) / GreatRadius);

    std::cout << inTakePipeA1 << std::endl;
    std::cout << inTakePipeA2 << std::endl;

    std::cout << outTakePipeA1 << std::endl;
    std::cout << outTakePipeA2 << std::endl;


    float PipeThickness = sy*0.02f;

    Rectangle inTakeUpRect = {-0.1*sx, inTakeUp - PipeThickness, Stator.x + GreatRadius*cos(inTakePipeA1) + sx*0.1f, PipeThickness};
    Rectangle inTakeDwRect = {-0.1*sx, inTakeDw,                 Stator.x + GreatRadius*cos(inTakePipeA2) + sx*0.1f, PipeThickness};

    Rectangle outTakeUpRect = {-0.1*sx, outTakeUp - PipeThickness, Stator.x + GreatRadius*cos(outTakePipeA1) + sx*0.085f, PipeThickness};
    Rectangle outTakeDwRect = {-0.1*sx, outTakeDw + 8,                 Stator.x + GreatRadius*cos(outTakePipeA2) + sx*0.1f, PipeThickness};

    // Rectangle outTakeUpRect = {Stator.x + GreatRadius*cos(outTakePipeA1), outTakeUp,
    //                            sx - Stator.x + GreatRadius*cos(outTakePipeA1) + sx*0.1f, PipeThickness};
    // Rectangle outTakeDwRect = {Stator.x + GreatRadius*cos(outTakePipeA2), outTakeDw,
    //                            sx - Stator.x + GreatRadius*cos(outTakePipeA2) + sx*0.1f, PipeThickness};

    std::vector<Particle> ParticleArray(N_PARTICLES_CONST);  
    std::vector<Pallete>  Palletes(N_PALLETES);

    for (int i = 0; i < N_PARTICLES_CONST; i++) {
        
        Vector2 pos;
        float delta = (GreatRadiusOut - GreatRadius) / 2.0f;
        float d;

        do {
            pos = {RanNorm() * sx, RanNorm() * sy};
            d = Vector2Distance(pos, Stator);
        } while (fabs(d - GreatRadius - delta) < delta + 2*radius);
        //} while (d + radius > GreatRadius);

        float angle = RanNorm()*2*PI;
        Vector2 vel = {speed*cos(angle), speed*sin(angle)};

        ParticleArray[i] = {pos, vel, radius, mass, RED};
    }

    InitWindow(sx, sy, "simulation");
    SetTargetFPS(T_FPS);

    int n_frames = 0;
    int N_PARTICLES = N_PARTICLES_CONST;

    while (!WindowShouldClose()) {

        n_frames++;

        if ( n_frames % 5 == 0) {
            
            float NewPclAngle = RanNorm() * RanSign() * 30.0f*PI/180.0f;
            Particle NewPcl;
            
            NewPcl.p = {-0.05*sx, Lerp(inTakeUp, inTakeDw, RanNorm())};
            NewPcl.v = Vector2Scale({cos(NewPclAngle), sin(NewPclAngle)}, speed);
            NewPcl.r = radius;
            NewPcl.m = 1;
            NewPcl.c = RED;
            
            ParticleArray.push_back(NewPcl);
            N_PARTICLES++;

            //std::cout << ParticleArray[0].p.x << " / " << ParticleArray[0].p.y << std::endl;
        }

        /* PALETAS */

        for (int i = 0; i < N_PALLETES; i++) {
            float alpha = mod(-GetTime()*ANG_V + 2*PI*i/N_PALLETES, 2*PI);

            Vector2 p1, p2;
            p1 = Vector2Add(Rotor, Vector2Scale({cos(alpha), -sin(alpha)}, 0.9f*RotorRadius));
            //DrawCircleV(p1, sy*0.01f, BLUE);

            float a = 1 + tan(alpha)*tan(alpha);
            float b = 2 * (Stator.y - Rotor.y) * tan(alpha);
            float c = (Stator.y - Rotor.y)*(Stator.y - Rotor.y) - GreatRadius*GreatRadius;

            float phi = -asin((Stator.y - Rotor.y) / GreatRadius);

            //std::cout << "alpha: " << alpha << " / " << "phi: " << phi << std::endl;

            if (phi < alpha && alpha < PI/2.0f) {
                float bhaskara = (-b + sqrt(b*b - 4*a*c)) / (2*a);
                p2 = {bhaskara, -GreatRadius*sin(acos(bhaskara / GreatRadius))};

            } else if (PI/2.0f < alpha && alpha < PI - phi) {
                float bhaskara = (-b - sqrt(b*b - 4*a*c)) / (2*a);
                p2 = {bhaskara, -GreatRadius*sin(acos(bhaskara / GreatRadius))};

            } else if (PI - phi < alpha && alpha < 3*PI/2.0f) {
                float bhaskara = (-b - sqrt(b*b - 4*a*c)) / (2*a);
                p2 = {bhaskara, GreatRadius*sin(acos(bhaskara / GreatRadius))};

            } else if ((0.0f < alpha && alpha < phi) || (3*PI/2.0f < alpha && alpha < 2*PI)) {
                float bhaskara = (-b + sqrt(b*b - 4*a*c)) / (2*a);
                p2 = {bhaskara, GreatRadius*sin(acos(bhaskara / GreatRadius))};
            }

            Palletes[i].p1 = p1;
            Palletes[i].p2 = Vector2Add(Stator, p2);
        }

        for (int step = 0; step < SUBSTEPS; step++) {

            /* PARTICULAS */

            for (int i = 0; i < N_PARTICLES; i++) {

                Particle* pcl = &ParticleArray[i];

                ParticleArray[i].p = Vector2Add(pcl->p, Vector2Scale(pcl->v, 1/(T_FPS*SUBSTEPS)));

                // particle collision
                for (int j = i + 1; j < N_PARTICLES; j++) {

                    Particle* pcl2 = &ParticleArray[j];

                    if (IsTouching(*pcl, *pcl2)) {

                        Vector2 delta = Vector2Subtract(pcl->p, pcl2->p);
                        float dist = Vector2Length(delta);
                        float overlap = pcl->r + pcl2->r - dist;

                        if (overlap > 0) {
                            Vector2 correction = Vector2Scale(Vector2Normalize(delta), overlap / 2.0f);
                            pcl->p = Vector2Add(pcl->p, correction);
                            pcl2->p = Vector2Subtract(pcl2->p, correction);
                        }

                        pcl ->v = ElasticCollision(*pcl, *pcl2);
                        pcl2->v = ElasticCollision(*pcl2, *pcl);
                    }
                }


                if (Vector2Length(pcl->v) > V_MAX) {
                    pcl->v = Vector2Scale(pcl->v, V_MAX / Vector2Length(pcl->v));
                }

                // wall collision x = 0
                if (pcl->p.x - pcl->r < -0.1f*sx) {
                    ParticleArray.erase(ParticleArray.begin() + i);
                    N_PARTICLES--;
                    continue;
                }
                // wall collision x = sx
                if (pcl->p.x + pcl->r > sx*1.1f) {
                    ParticleArray.erase(ParticleArray.begin() + i);
                    N_PARTICLES--;
                    continue;
                }

                // wall collision y = 0
                if (pcl->p.y - pcl->r < 0) {
                    ParticleArray.erase(ParticleArray.begin() + i);
                    N_PARTICLES--;
                    continue;
                }
                // wall collision y = sy
                if (pcl->p.y + pcl->r > sy) {
                    ParticleArray.erase(ParticleArray.begin() + i);
                    N_PARTICLES--;
                    continue;
                }

                // ROTOR COLLISION

                if (Vector2Distance(pcl->p, Rotor) < RotorRadius + pcl->r) {
                    Vector2 delta = Vector2Subtract(pcl->p, Rotor);
                    float dist = Vector2Length(delta);
                    float overlap = pcl->r + RotorRadius - dist;

                    if (overlap > 0) {
                        Vector2 correction = Vector2Scale(Vector2Normalize(delta), overlap / 2.0f);
                        pcl->p = Vector2Add(pcl->p, correction);
                    }

                    pcl ->v = StaticCollisionCircle(Rotor, *pcl);
                }

                // PALLETES COLLITIONS

                for (int k = 0; k < N_PALLETES; k++) {
                    Vector2 normal;
                    Vector2* p1 = &Palletes[k].p1;
                    Vector2* p2 = &Palletes[k].p2;

                    Vector2 np2 = Vector2Subtract(*p2, Rotor);
                    np2 = Vector2Scale(Vector2Normalize(np2), Vector2Distance(*p2, Rotor) - palleteThickness*0.5f);
                    np2 = Vector2Add(np2, Rotor);

                    if (IsCollidingWithThickLine(*pcl, *p1, np2, palleteThickness, &normal)) {
                        // Calcular punto más cercano sobre el segmento
                        Vector2 seg = Vector2Subtract(*p2, *p1);
                        float seg_len_sq = Vector2LengthSqr(seg);
                        float t = Vector2DotProduct(Vector2Subtract(pcl->p, *p1), seg) / seg_len_sq;
                        t = fmaxf(0.0f, fminf(1.0f, t));  // clamp
                        Vector2 closest = Vector2Add(*p1, Vector2Scale(seg, t));
                    
                        // Corrige posición fuera del pallete (proyección + grosor)
                        Vector2 to_circle = Vector2Subtract(pcl->p, closest);
                        float dist = Vector2Length(to_circle);
                        float targetDist = pcl->r + palleteThickness / 2.0f;
                        float penetration = targetDist - dist;
                    
                        if (penetration > 0.0f) {
                            pcl->p = Vector2Add(pcl->p, Vector2Scale(normal, penetration));
                        }
                    
                        Vector2 r = Vector2Subtract(pcl->p, Rotor);          // Vector desde el centro a la partícula
                        Vector2 tangent = {-r.y, r.x};                       // Perpendicular antihoraria (si rotación es positiva)
                        Vector2 PaddleV = Vector2Scale(Vector2Normalize(tangent), Vector2Length(r) * ANG_V);
                        Vector2 v_rel = Vector2Subtract(pcl->v, PaddleV);
                        float vn = Vector2DotProduct(v_rel, normal);
                        if (vn < 0.0f) {
                            v_rel = Vector2Subtract(v_rel, Vector2Scale(normal, 2.0f * vn));
                        }
                        pcl->v = Vector2Add(v_rel, PaddleV); // volver al sistema global
                    }
                }

                // PIPE COLLISIONS

                // INTEAKE UP

                HandleRectCollision(*pcl, inTakeUpRect);
                HandleRectCollision(*pcl, inTakeDwRect);

                HandleRectCollision(*pcl, outTakeUpRect);
                HandleRectCollision(*pcl, outTakeDwRect);

                float gamma = atan2(pcl->p.y - Stator.y, pcl->p.x - Stator.x);
                if (gamma < 0) gamma += 2*PI;

                // if      (gamma >  inTakePipeA2 && gamma <  inTakePipeA1) { pcl->c =  BLUE; }
                // else if (gamma > outTakePipeA2 && gamma < /*2*PI*/ + outTakePipeA1) { pcl->c = GREEN; }
                // else pcl->c = RED;

                // STATOR COLLISIONS
                float d = Vector2Distance(pcl->p, Stator);
                if      (gamma >  inTakePipeA2 && gamma <  inTakePipeA1);
                else if (gamma > outTakePipeA2 && gamma < outTakePipeA1);
                else if (IsPointInRect(pcl->p, {-0.2*sx, outTakeUp - PipeThickness, 0.9*sx, outTakeDw - outTakeUp + 2*PipeThickness + 8}));
                else if (d > GreatRadius - pcl->r) {
                    pcl->p = Vector2Scale({cos(gamma), sin(gamma)}, GreatRadius - pcl->r);
                    pcl->p = Vector2Add(pcl->p, Stator);
                    pcl->v = StaticCollisionCircle(Stator, *pcl);
                }

                if (IsPointInRect(pcl->p, {0, outTakeUp, sx*0.5f, outTakeDw - outTakeUp + 8})) {
                    pcl->v = Vector2Scale(Vector2Normalize(pcl->v), speed);
                }

                // if (fabs(d - GreatRadius - StatorDelta) < StatorDelta) {
                // // if (!(d < GreatRadius + pcl->r || d > GreatRadiusOut - pcl->r)) {

                //     float gamma = atan2(pcl->p.y - Stator.y, pcl->p.x - Stator.x);
                //     if (gamma < 0) gamma += 2*PI;
                //     // float epsilon = atan2(GetMouseY() - Stator.y, GetMouseX() - Stator.x);
                //     // if (epsilon < 0) {
                //     //     epsilon += 2*PI;
                //     // }
                //     // std::cout << epsilon << " = " << epsilon/PI << "pi" << std::endl;

                //     if      (gamma >  inTakePipeA2 && gamma <  inTakePipeA1);
                //     else if (gamma > outTakePipeA2 && gamma < outTakePipeA1);

                //     // -- CHOQUE EXTERNO --
                //     else if (d > GreatRadius + StatorDelta) {
                //         pcl->p = Vector2Scale({cos(gamma), sin(gamma)}, GreatRadiusOut + 0*pcl->r);
                //         pcl->p = Vector2Add(pcl->p, Stator);
                //         pcl->v = StaticCollisionCircle(Stator, *pcl);
                //     } 
                //     // -- CHOQUE INTERNO --
                //     else if (d < GreatRadius + StatorDelta) {
                //         pcl->p = Vector2Scale({cos(gamma), sin(gamma)}, GreatRadius - 0*pcl->r);
                //         pcl->p = Vector2Add(pcl->p, Stator);
                //         pcl->v = StaticCollisionCircle(Stator, *pcl);
                //     }

                    // if (gamma < inTakeDw && gamma > inTakeUp) pcl->c = BLUE; 
                    // else if (gamma < outTakeDw) pcl->c = GREEN; 
                    // else pcl->c = RED;

                    

                    // Vector2 deltaVec = Vector2Subtract(pcl->p, Stator);
                    // float dist = Vector2Length(deltaVec);

                    // // Chequeamos si está adentro del borde interno
                    // float innerRadius = GreatRadius;
                    // float outerRadius = GreatRadiusOut;
                    // float overlap = 0;



                    // if (dist < innerRadius + pcl->r) {
                    //     // choque interior
                    //     overlap = (innerRadius + pcl->r) - dist;
                    //     Vector2 correction = Vector2Scale(Vector2Normalize(deltaVec), -overlap / 2.0f); // signo negativo
                    //     pcl->p = Vector2Add(pcl->p, correction);
                    // }
                    // else if (dist > outerRadius - pcl->r) {
                    //     // choque exterior
                    //     overlap = (pcl->r + outerRadius) - dist;
                    //     Vector2 correction = Vector2Scale(Vector2Normalize(deltaVec), overlap / 2.0f);
                    //     pcl->p = Vector2Add(pcl->p, correction);
                    // }

                    // // Solo corregimos velocidad si hay colisión
                    // if (overlap > 0) {
                    //     pcl->v = StaticCollisionCircle(Stator, *pcl);
                    // }
            }
        }

        BeginDrawing();

        ClearBackground(BLACK);

        DrawFPS(0,0);

        //std::cout << outTakePipeA2 << " - " << inTakePipeA2 << std::endl;
        DrawCircleV(Stator, GreatRadiusOut, DARKGRAY);

        //DrawCircleSector(Stator, GreatRadiusOut, outTakePipeA1*180/PI, inTakePipeA2*180/PI, 0, DARKGRAY);
        // DrawCircleSector(Stator, GreatRadiusOut, inTakePipeA1*180/PI, 180 + outTakePipeA2*180/PI, 0, DARKGRAY);

        // DrawRectangle(0, inTakeUp - sy*0.05f, sx*0.5f, inTakeDw - inTakeUp + 2*sy*0.05f, ORANGE);
        // DrawRectangle(sx*0.5f, outTakeUp - sy*0.05f, sx*0.5f, outTakeDw - outTakeUp + 2*sy*0.05f, BLUE);
        
        DrawCircleV(Stator, GreatRadius, GRAY);
        DrawRectangle(0, inTakeUp, sx*0.7f, inTakeDw - inTakeUp, GRAY);
        DrawRectangle(0, outTakeUp, sx*0.7f, outTakeDw - outTakeUp + 8, GRAY);
        DrawTriangle({Stator.x, Stator.y + GreatRadius}, 
                     {Stator.x, Stator.y + GreatRadius + 8},
                     {Stator.x + GreatRadius*cos(80*PI/180.0f), Stator.y + GreatRadius*sin(80*PI/180.0f)},
                      GRAY);
        
        DrawCircleV(Rotor, RotorRadius, WHITE);
        DrawCircleV(Rotor, sy*0.02f, GRAY);    
        
        DrawRectangleRec(inTakeUpRect,  DARKGRAY);
        DrawRectangleRec(inTakeDwRect,  DARKGRAY);
        DrawRectangleRec(outTakeUpRect, DARKGRAY);
        DrawRectangleRec(outTakeDwRect, DARKGRAY);

        // DrawRectangleRec({0, outTakeUp, sx*0.5f, outTakeDw - outTakeUp + 8}, BLUE);

        // DrawRectangleRec({-0.2*sx, outTakeUp - PipeThickness, 0.9*sx, outTakeDw - outTakeUp + 2*PipeThickness + 8}, ORANGE);

        for (int i = 0; i < N_PALLETES; i++) {
            DrawLineEx(Palletes[i].p1, Palletes[i].p2, sy*0.05f, WHITE);
        }

        for (int i = 0; i < N_PARTICLES; i++) {
            Particle* pcl = &ParticleArray[i];

            DrawCircleV(pcl->p, pcl->r, pcl->c);
        }

        const char* NParticlesChar = TextFormat("%d", (int)ParticleArray.size());
        DrawText(NParticlesChar, sx - MeasureText(NParticlesChar, sy*0.05f), 0, sy*0.05f, WHITE);

        EndDrawing();
    }
    
    return 0;
}