/*
 * Robot auto-équilibré — ESP32 NodeMCU-32S
 * TB6612FNG + 2 × JGA25-370 (encodeurs) + MPU6050, interface web.
 *
 * Loi de commande (toutes les grandeurs : positif = vers l'AVANT) :
 *   u = Kp·e + Kd·ω + Kv·v + Kx·x
 *   e = θ − θ₀ (°), ω (°/s), v = vitesse moyenne des roues (imp/s),
 *   x = déplacement depuis l'armement (imp).
 *   Les 4 gains sont POSITIFS. Kv > 0 et Kx > 0 sont indispensables : un PD
 *   seul sur l'angle laisse un mode lent instable (le robot part en 2-4 s).
 *
 * Tous les réglages se modifient depuis la page (appliqués tout de suite,
 * « Sauvegarder » les garde en flash). Seuls brochage et WiFi sont ici.
 *
 * Mise en route, dans l'ordre :
 *  1. Robot IMMOBILE ~1 s à l'allumage (calibration gyro).
 *     WiFi "Robot-Bal" (mdp changeme123) -> http://192.168.4.1
 *  2. Roues en l'air, « Test moteurs » : ▶ doit faire avancer la bonne roue
 *     ET monter son encodeur (sinon : carte « Moteurs & encodeurs »).
 *  3. Pencher le robot vers l'AVANT : θ doit devenir POSITIF
 *     (sinon « Inverser l'angle »).
 *  4. « Modèle → gains » : Identifier moteur (roues en l'air, sur batterie),
 *     mesurer N, renseigner r et l, Calculer, Appliquer.
 *  5. Tenir le robot à son équilibre : « Mesurer θ₀ ». ARMER, vérifier qu'il tient.
 *  6. Autotune si besoin : il termine par un essai long de validation et
 *     remet les gains de départ si les nouveaux ne tiennent pas. STOP, Sauvegarder.
 *
 * Téléversement par WiFi (après un 1er téléversement USB, schéma de partition
 * avec OTA, ex. « Default 4MB with spiffs ») :
 *  - Arduino IDE : WiFi "Robot-Bal", port réseau 192.168.4.1, mdp changeme123 ;
 *  - ou page web, carte « Mise à jour du firmware » : fichier .ino.bin.
 *
 * Core Arduino-ESP32 3.x. Aucune bibliothèque externe.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include "soc/gpio_reg.h"

// ---------------- Brochage / WiFi (fixes) ----------------
const int PWMA = 25, AIN1 = 26, AIN2 = 27;   // canal A du TB6612
const int PWMB = 33, BIN1 = 32, BIN2 = 13;   // canal B du TB6612
const int STBY = 4;
const int ENC1_A = 16, ENC1_B = 17;
const int ENC2_A = 18, ENC2_B = 19;
const int SDA_PIN = 21, SCL_PIN = 22;
const int PWM_RES = 8;                       // 0..255
const uint8_t MPU = 0x68;
// À changer avant de flasher : mot de passe du point d'accès et de l'OTA (8 caractères min.).
const char *AP_SSID  = "Robot-Bal";
const char *AP_PASS  = "changeme123";
const char *OTA_HOST = "robot-bal";
const char *OTA_PASS = "changeme123";

// ======================================================================
// Paramètres modifiables
// ======================================================================
const int NG = 4;                    // gains : Kp, Kd, Kv, Kx
float G[NG];
float &Kp = G[0], &Kd = G[1], &Kv = G[2], &Kx = G[3];
float theta0, vTauMs;
int   deadband;
bool  invAngle;
float fallDeg, armDeg; int armHoldMs;
bool  useRoll; float compK; int loopMs;
bool  swapMot, swapEnc, invMotL, invMotR, invEncL, invEncR;
int   pwmFreq, testPwmSet, testMs, nImp;
float kmM, u0M, tauMs, rMm, lMm, lFac, lqX, lqT, lqU; int idMax;
bool  tuneSel[NG];
float settleS, measureS, validS, stepFrac, tol, minImpr;
float wRate, wSat, wV, wX, wGrow, fallJ, stepMin[NG];
int   maxIter;

enum PType : uint8_t { T_F, T_I, T_B };
enum PLock : uint8_t { L_NONE, L_TUNE, L_IDLE };   // L_TUNE : figé pendant l'autotune ; L_IDLE : modifiable à l'arrêt seulement
struct Param {
  const char *key, *label, *group, *hint;
  PType t; void *p; float def, mn, mx; PLock lock;
};

Param PARAMS[] = {
  // --- Régulateur ---
  {"kp",   "Kp", "Régulateur", "PWM par ° d'erreur d'angle", T_F, &G[0], 25, 0, 2000, L_TUNE},
  {"kd",   "Kd", "Régulateur", "PWM par °/s de vitesse de bascule", T_F, &G[1], 0.8f, 0, 200, L_TUNE},
  {"kv",   "Kv", "Régulateur", "PWM par imp/s de vitesse des roues (> 0)", T_F, &G[2], 0, 0, 10, L_TUNE},
  {"kx",   "Kx", "Régulateur", "PWM par impulsion de déplacement (> 0)", T_F, &G[3], 0, 0, 10, L_TUNE},
  {"t0",   "θ₀ consigne (°)", "Régulateur", "angle d'équilibre ; bouton Mesurer θ₀", T_F, &theta0, 0, -45, 45, L_NONE},
  {"ia",   "Inverser l'angle", "Régulateur", "θ doit être positif quand le robot penche vers l'avant", T_B, &invAngle, 0, 0, 1, L_IDLE},
  {"db",   "Zone morte moteur (PWM)", "Régulateur", "compensation progressive sur ±db ; = u₀ après identification", T_I, &deadband, 0, 0, 200, L_NONE},
  {"vtau", "Filtre vitesse roues (ms)", "Régulateur", "lissage de v ; 0 = aucun", T_F, &vTauMs, 15, 0, 200, L_NONE},
  // --- Sécurité ---
  {"fall",  "Angle de chute (°)", "Sécurité", "au-delà, moteurs coupés", T_F, &fallDeg, 35, 5, 90, L_NONE},
  {"arm",   "Angle de relance (°)", "Sécurité", "|e| sous ce seuil pour réarmer", T_F, &armDeg, 3, 0.5f, 20, L_NONE},
  {"armms", "Durée de relance (ms)", "Sécurité", "temps à rester droit avant de repartir", T_I, &armHoldMs, 500, 0, 5000, L_NONE},
  // --- Capteur ---
  {"roll",   "Bascule sur l'axe roll (gx)", "Capteur MPU6050", "décoché = pitch (gy)", T_B, &useRoll, 0, 0, 1, L_IDLE},
  {"compk",  "Filtre complémentaire K", "Capteur MPU6050", "part du gyro ; 0,995 à 200 Hz (τ ≈ 1 s)", T_F, &compK, 0.995f, 0.5f, 0.999f, L_NONE},
  {"loopms", "Période de boucle (ms)", "Capteur MPU6050", "5 ms = 200 Hz", T_I, &loopMs, 5, 2, 20, L_IDLE},
  // --- Moteurs & encodeurs ---
  {"swm",  "Échanger moteurs G/D", "Moteurs & encodeurs", "Test gauche fait tourner la roue droite", T_B, &swapMot, 0, 0, 1, L_IDLE},
  {"iml",  "Inverser moteur gauche", "Moteurs & encodeurs", "roue gauche part en arrière sur ▶", T_B, &invMotL, 0, 0, 1, L_IDLE},
  {"imr",  "Inverser moteur droit", "Moteurs & encodeurs", "roue droite part en arrière sur ▶", T_B, &invMotR, 0, 0, 1, L_IDLE},
  {"swe",  "Échanger encodeurs G/D", "Moteurs & encodeurs", "l'encodeur gauche bouge avec la roue droite", T_B, &swapEnc, 0, 0, 1, L_IDLE},
  {"iel",  "Inverser encodeur gauche", "Moteurs & encodeurs", "compte en négatif quand la roue avance", T_B, &invEncL, 0, 0, 1, L_IDLE},
  {"ier",  "Inverser encodeur droit", "Moteurs & encodeurs", "compte en négatif quand la roue avance", T_B, &invEncR, 0, 0, 1, L_IDLE},
  {"pwmf", "Fréquence PWM (Hz)", "Moteurs & encodeurs", "20 000 = inaudible", T_I, &pwmFreq, 20000, 1000, 40000, L_IDLE},
  {"tpwm", "PWM des tests", "Moteurs & encodeurs", "", T_I, &testPwmSet, 80, 0, 255, L_NONE},
  {"tms",  "Durée des tests (ms)", "Moteurs & encodeurs", "", T_I, &testMs, 1000, 100, 5000, L_NONE},
  // --- Modèle (identification + LQR) ---
  {"nimp", "N : impulsions par tour de roue", "Modèle (pour le calcul des gains)", "bouton « N = compte gauche »", T_I, &nImp, 0, 0, 100000, L_NONE},
  {"rmm",  "Rayon de roue r (mm)", "Modèle (pour le calcul des gains)", "diamètre / 2", T_F, &rMm, 32.5f, 5, 200, L_NONE},
  {"lmm",  "Hauteur centre de masse l (mm)", "Modèle (pour le calcul des gains)", "robot couché sur un crayon : axe des roues → point d'équilibre", T_F, &lMm, 0, 0, 1000, L_NONE},
  {"lf",   "Facteur l_eff / l", "Modèle (pour le calcul des gains)", "corps non ponctuel ; 1,2 typique", T_F, &lFac, 1.2f, 1, 2, L_NONE},
  {"km",   "K_m moteur ((imp/s)/PWM)", "Modèle (pour le calcul des gains)", "rempli par Identifier moteur", T_F, &kmM, 0, 0, 10000, L_NONE},
  {"u0",   "u₀ zone morte (PWM)", "Modèle (pour le calcul des gains)", "rempli par Identifier moteur", T_F, &u0M, 0, 0, 255, L_NONE},
  {"taum", "τ moteur (ms)", "Modèle (pour le calcul des gains)", "rempli par Identifier moteur", T_F, &tauMs, 0, 0, 2000, L_NONE},
  {"idmax","PWM max de l'identification", "Modèle (pour le calcul des gains)", "paliers à 40 %, 70 %, 100 %", T_I, &idMax, 220, 60, 255, L_NONE},
  {"lqx",  "Déplacement toléré (cm)", "Modèle (pour le calcul des gains)", "plus grand = Kx plus faible", T_F, &lqX, 20, 1, 500, L_NONE},
  {"lqt",  "Angle toléré (°)", "Modèle (pour le calcul des gains)", "plus grand = gains plus doux", T_F, &lqT, 3, 0.2f, 30, L_NONE},
  {"lqu",  "Commande max visée (PWM)", "Modèle (pour le calcul des gains)", "plus petit = gains plus doux", T_F, &lqU, 150, 10, 255, L_NONE},
  // --- Autotune ---
  {"tkp",    "Régler Kp", "Autotune", "", T_B, &tuneSel[0], 1, 0, 1, L_TUNE},
  {"tkd",    "Régler Kd", "Autotune", "", T_B, &tuneSel[1], 1, 0, 1, L_TUNE},
  {"tkv",    "Régler Kv", "Autotune", "", T_B, &tuneSel[2], 1, 0, 1, L_TUNE},
  {"tkx",    "Régler Kx", "Autotune", "", T_B, &tuneSel[3], 1, 0, 1, L_TUNE},
  {"settle", "Stabilisation (s)", "Autotune", "début d'essai non compté", T_F, &settleS, 2, 0, 10, L_TUNE},
  {"meas",   "Mesure (s)", "Autotune", "coupée en 2 moitiés pour détecter une oscillation qui grossit", T_F, &measureS, 12, 2, 60, L_TUNE},
  {"valid",  "Validation finale (s)", "Autotune", "essai long des gains trouvés ; 0 = aucun", T_F, &validS, 40, 0, 300, L_TUNE},
  {"iters",  "Itérations max", "Autotune", "", T_I, &maxIter, 12, 1, 100, L_TUNE},
  {"frac",   "Pas initial (fraction du gain)", "Autotune", "", T_F, &stepFrac, 0.2f, 0.01f, 1, L_TUNE},
  {"tol",    "Tolérance d'arrêt", "Autotune", "somme des pas relatifs", T_F, &tol, 0.1f, 0.001f, 1, L_TUNE},
  {"impr",   "Amélioration minimale", "Autotune", "fraction de J à gagner pour accepter un essai (anti-bruit)", T_F, &minImpr, 0.02f, 0, 0.5f, L_TUNE},
  {"wrate",  "Poids de ω² dans J", "Autotune", "ω en °/s ; pénalise les tremblements", T_F, &wRate, 0.002f, 0, 1, L_TUNE},
  {"wv",     "Poids de v² dans J", "Autotune", "v en cm/s (N et r requis)", T_F, &wV, 0.01f, 0, 10, L_TUNE},
  {"wx",     "Poids de x² dans J", "Autotune", "x en cm depuis le début de l'essai (N et r requis)", T_F, &wX, 0.002f, 0, 10, L_TUNE},
  {"wsat",   "Poids saturation dans J", "Autotune", "", T_F, &wSat, 5, 0, 1000, L_TUNE},
  {"wgrow",  "Poids de la croissance dans J", "Autotune", "× (J 2e moitié − J 1re moitié) si positif", T_F, &wGrow, 2, 0, 100, L_TUNE},
  {"fallj",  "Coût d'une chute", "Autotune", "", T_F, &fallJ, 1000, 10, 100000, L_TUNE},
  {"smp",    "Pas min Kp", "Autotune", "", T_F, &stepMin[0], 1, 0.0001f, 100, L_TUNE},
  {"smd",    "Pas min Kd", "Autotune", "", T_F, &stepMin[1], 0.05f, 0.0001f, 100, L_TUNE},
  {"smv",    "Pas min Kv", "Autotune", "", T_F, &stepMin[2], 0.005f, 0.00001f, 10, L_TUNE},
  {"smx",    "Pas min Kx", "Autotune", "", T_F, &stepMin[3], 0.002f, 0.00001f, 10, L_TUNE},
};
const int NPARAM = sizeof(PARAMS) / sizeof(PARAMS[0]);

float getP(const Param &p) {
  switch (p.t) {
    case T_F: return *(float *)p.p;
    case T_I: return *(int *)p.p;
    default:  return *(bool *)p.p ? 1 : 0;
  }
}
void setP(const Param &p, float v) {
  v = constrain(v, p.mn, p.mx);
  switch (p.t) {
    case T_F: *(float *)p.p = v; break;
    case T_I: *(int *)p.p = lroundf(v); break;
    default:  *(bool *)p.p = v > 0.5f; break;
  }
}
Param *findP(const char *k) {
  for (int i = 0; i < NPARAM; i++) if (!strcmp(k, PARAMS[i].key)) return &PARAMS[i];
  return nullptr;
}

Preferences prefs;
WebServer server(80);
volatile bool dirty = false;

void loadParams() {
  for (int i = 0; i < NPARAM; i++) {
    Param &p = PARAMS[i];
    switch (p.t) {
      case T_F: setP(p, prefs.getFloat(p.key, p.def)); break;
      case T_I: setP(p, prefs.getInt(p.key, lroundf(p.def))); break;
      default:  setP(p, prefs.getBool(p.key, p.def > 0.5f) ? 1 : 0); break;
    }
  }
}
void saveParams() {
  for (int i = 0; i < NPARAM; i++) {
    Param &p = PARAMS[i];
    switch (p.t) {
      case T_F: prefs.putFloat(p.key, *(float *)p.p); break;
      case T_I: prefs.putInt(p.key, *(int *)p.p); break;
      default:  prefs.putBool(p.key, *(bool *)p.p); break;
    }
  }
  dirty = false;
}
float cmPerImp() { return nImp > 0 ? 2 * PI * rMm / 10.0f / nImp : 0; }

// ---------------- État partagé ----------------
enum Mode : uint8_t { IDLE = 0, WAIT_UP = 1, BALANCE = 2 };
volatile Mode mode = IDLE;
volatile bool reqArm = false, reqDisarm = false, reqTune = false, reqStop = false;
volatile bool reqT0 = false, reqCal = false, reqPwm = false, reqEncReset = false, reqIdent = false;
volatile bool otaActive = false;     // mise à jour en cours : moteurs coupés

volatile float    telTheta = 0, telRate = 0, telV = 0, telX = 0;
volatile int      telU = 0;
volatile uint32_t telLoopUs = 0, falls = 0;
volatile bool     mpuOk = false;

// ---------------- Moteurs ----------------
void setMotor(int pwm, int in1, int in2, int v, bool inv) {
  v = constrain(v, -255, 255);
  if (inv) v = -v;
  digitalWrite(in1, v > 0 ? HIGH : LOW);
  digitalWrite(in2, v < 0 ? HIGH : LOW);
  ledcWrite(pwm, abs(v));
}
void drive(int l, int r) {           // positif = avant
  if (swapMot) { setMotor(PWMB, BIN1, BIN2, l, invMotL); setMotor(PWMA, AIN1, AIN2, r, invMotR); }
  else         { setMotor(PWMA, AIN1, AIN2, l, invMotL); setMotor(PWMB, BIN1, BIN2, r, invMotR); }
}
// Compensation de zone morte CONTINUE : pente 2 sur |u| < db, puis u ± db.
// (L'ancien saut 0 → ±db au moindre |u| > 0,5 agissait comme un relais et
//  entretenait des oscillations autour de l'équilibre.)
int compensate(float u) {
  float db = deadband;
  float out = db > 0 ? u + db * constrain(u / db, -1.0f, 1.0f) : u;
  return constrain((int)lroundf(out), -255, 255);
}

// ---------------- Encodeurs (quadrature ×4) ----------------
volatile long enc1 = 0, enc2 = 0;
volatile uint8_t st1 = 0, st2 = 0;
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static inline uint8_t readPins(int a, int b) {
  uint32_t in = REG_READ(GPIO_IN_REG);
  return (((in >> a) & 1) << 1) | ((in >> b) & 1);
}
void IRAM_ATTR isr1() { uint8_t s = readPins(ENC1_A, ENC1_B); enc1 += QDEC[(st1 << 2) | s]; st1 = s; }
void IRAM_ATTR isr2() { uint8_t s = readPins(ENC2_A, ENC2_B); enc2 += QDEC[(st2 << 2) | s]; st2 = s; }
long encLeft()  { long v = swapEnc ? enc2 : enc1; return invEncL ? -v : v; }
long encRight() { long v = swapEnc ? enc1 : enc2; return invEncR ? -v : v; }
volatile float speedL = 0, speedR = 0;   // affichage, sur 100 ms

volatile int testMask = 0, testDir = 1;  // bit 0 = gauche, bit 1 = droite
volatile uint32_t testUntil = 0;

// ---------------- MPU6050 ----------------
const float GYRO_LSB = 65.5f;            // ±500 °/s (±250 saturait pendant les oscillations)
float gxOff = 0, gyOff = 0, theta = 0, rate = 0, lastGx = 0, lastGy = 0;
uint32_t lastImuUs = 0;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
bool mpuInit() {
  if (!writeReg(0x6B, 0x00)) return false;   // réveil
  delay(50);
  return writeReg(0x1A, 0x03)                // passe-bas ~44 Hz
      && writeReg(0x1B, 0x08)                // gyro ±500 °/s
      && writeReg(0x1C, 0x00);               // accéléro ±2 g
}
bool readRaw(float &ax, float &ay, float &az, float &gx, float &gy) {
  uint8_t b[14];
  if (!readRegs(0x3B, b, 14)) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  ax = r[0] / 16384.0f; ay = r[1] / 16384.0f; az = r[2] / 16384.0f;
  gx = r[4] / GYRO_LSB; gy = r[5] / GYRO_LSB;
  return true;
}
float accAngle(float ax, float ay, float az) {
  return useRoll ? atan2f(ay, az) * RAD_TO_DEG
                 : atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}
void calibrateBlocking() {
  float ax, ay, az, gx, gy, sx = 0, sy = 0;
  int n = 0;
  for (int i = 0; i < 400; i++) {
    if (readRaw(ax, ay, az, gx, gy)) { sx += gx; sy += gy; n++; }
    delay(2);
  }
  if (n) { gxOff = sx / n; gyOff = sy / n; }
  if (readRaw(ax, ay, az, gx, gy)) theta = accAngle(ax, ay, az);
  lastImuUs = micros();
}
bool updateImu() {
  float ax, ay, az, gx, gy;
  if (!readRaw(ax, ay, az, gx, gy)) return false;
  lastGx = gx; lastGy = gy;
  rate = useRoll ? gx - gxOff : gy - gyOff;
  uint32_t now = micros();
  float dt = (now - lastImuUs) * 1e-6f;
  lastImuUs = now;
  theta = compK * (theta + rate * dt) + (1 - compK) * accAngle(ax, ay, az);
  return true;
}

// ---------------- Identification moteur (roues en l'air) ----------------
// 3 paliers de PWM. Pour un 1er ordre v' = (v_f − v)/τ, en régime établi
// x(t) ≈ v_f·(t − τ) : une droite ajustée sur x(t) entre 0,8 et 1,5 s donne
// v_f (pente) et τ = −ordonnée/pente. Puis v_f = K_m·(u − u₀) sur les 3 paliers.
const int ID_N = 3;
volatile bool identRun = false, identWarn = false;
volatile int  identStep = 0, identErr = 0;   // err : 1 sens, 2 pente, 3 annulé
int    idU[ID_N];
float  idV[ID_N], idTau[ID_N];
bool   idPause = false, idHave0 = false;
float  idT = 0, idBase = 0, idT0 = 0;
double sN, sT, sX, sTT, sTX;
long   idL0 = 0, idR0 = 0;

void identResetStep(float pos) {
  idPause = false; idT = 0; idBase = pos; idHave0 = false;
  sN = sT = sX = sTT = sTX = 0;
}
void identStart(float pos) {
  for (int i = 0; i < ID_N; i++) { idU[i] = lroundf(idMax * (0.4f + 0.3f * i)); idV[i] = 0; idTau[i] = 0; }
  identStep = 0; identErr = 0; identWarn = false;
  identResetStep(pos);
  identRun = true;
}
void identFinish() {
  identRun = false;
  for (int i = 0; i < ID_N; i++) if (idV[i] <= 0) { identErr = 1; return; }
  float mu = 0, mv = 0;
  for (int i = 0; i < ID_N; i++) { mu += idU[i]; mv += idV[i]; }
  mu /= ID_N; mv /= ID_N;
  float suu = 0, suv = 0;
  for (int i = 0; i < ID_N; i++) { suu += (idU[i] - mu) * (idU[i] - mu); suv += (idU[i] - mu) * (idV[i] - mv); }
  float a = suv / suu;
  if (a <= 0) { identErr = 2; return; }
  float u0 = max(0.0f, mu - mv / a);
  float ts = 0; int tn = 0;
  for (int i = 0; i < ID_N; i++) if (idTau[i] > 0) { ts += idTau[i]; tn++; }
  setP(*findP("km"), a);
  setP(*findP("u0"), u0);
  setP(*findP("taum"), tn ? 1000 * ts / tn : 50);
  setP(*findP("db"), u0);
  dirty = true;
}
int identTick(float pos, long cL, long cR, float dt) {   // renvoie la PWM
  idT += dt;
  if (idPause) {                                         // roues libres 1 s
    if (idT >= 1.0f) {
      if (++identStep >= ID_N) identFinish();
      else identResetStep(pos);
    }
    return 0;
  }
  if (idT >= 0.8f) {
    double t = idT, x = pos - idBase;
    sN++; sT += t; sX += x; sTT += t * t; sTX += t * x;
    if (!idHave0) { idL0 = cL; idR0 = cR; idT0 = idT; idHave0 = true; }
  }
  if (idT >= 1.5f) {
    double den = sN * sTT - sT * sT;
    float slope = den > 0 ? (sN * sTX - sT * sX) / den : 0;
    float icpt  = sN > 0 ? (sX - slope * sT) / sN : 0;
    idV[identStep] = slope;
    idTau[identStep] = slope > 1 ? -icpt / slope : 0;
    if (identStep == ID_N - 1 && idT > idT0) {
      float vl = (cL - idL0) / (idT - idT0), vr = (cR - idR0) / (idT - idT0);
      identWarn = fabsf(vl - vr) > 0.15f * 0.5f * fabsf(vl + vr);
    }
    idPause = true; idT = 0;
    return 0;
  }
  return idU[identStep];
}

// ---------------- Autotune (Twiddle + validation) ----------------
// Chaque essai : stabilisation, puis mesure coupée en deux moitiés.
// J = moyenne(J1, J2) + wGrow·max(0, J2 − J1) : une oscillation qui grossit
// (même lentement) est pénalisée au lieu de passer inaperçue.
// À la fin, essai long de validation des meilleurs gains ; s'il chute ou
// si l'oscillation grossit, les gains de départ sont remis.
enum TState : uint8_t { TS_OFF, TS_RUN, TS_VALID, TS_OK, TS_FAIL, TS_STOPPED };
volatile uint8_t tstate = TS_OFF;
volatile bool  tuning = false;                 // TS_RUN ou TS_VALID
volatile int   ti = 0, tphase = 0, iter = 0;   // tphase : 0 réf., 1 +Δ, 2 −Δ, 3 validation
volatile float bestJ = 1e9f, trialT = 0;
float P[NG], DP[NG], bestP[NG], startP[NG];
double sumC[2]; int nC[2], nSat;
float trialX0 = 0; bool trialX0Set = false;

struct Trial { int16_t iter; int8_t param, phase; float g[NG]; float J; bool fell; };
const int LOG_N = 32;
Trial tlog[LOG_N];
volatile int logCount = 0;

void applyP(const float *p) { for (int i = 0; i < NG; i++) G[i] = p[i]; }
void beginTrial() { trialT = 0; sumC[0] = sumC[1] = 0; nC[0] = nC[1] = nSat = 0; trialX0Set = false; }
float trialLen() { return tstate == TS_VALID ? validS : measureS; }
bool gainsOkForTune() { return Kv > 0 && Kx > 0; }
bool allowed(int i, float c) { return i < 2 ? c >= 0 : c > 0; }   // Kv, Kx strictement positifs
bool converged() {
  float s = 0;
  for (int i = 0; i < NG; i++) if (tuneSel[i]) s += DP[i] / (fabsf(bestP[i]) + stepMin[i]);
  return s < tol;
}
void logTrial(float J, bool fell) {
  Trial &t = tlog[logCount % LOG_N];
  t.iter = iter; t.param = ti; t.phase = tphase;
  memcpy(t.g, P, sizeof P); t.J = J; t.fell = fell;
  logCount = logCount + 1;
}
void tunerStop() {                       // arrêt manuel : meilleurs gains trouvés
  if (!tuning) return;
  tuning = false; tstate = TS_STOPPED;
  applyP(bestP); dirty = true;
}
void tunerEndSearch() {                  // fin de la recherche → validation
  memcpy(P, bestP, sizeof P);
  applyP(bestP);
  if (validS > 0) { tstate = TS_VALID; tphase = 3; }
  else { tuning = false; tstate = TS_OK; dirty = true; }
}
void nextGain() {
  tphase = 1;
  if (++ti >= NG) {
    ti = 0; iter = iter + 1;
    if (iter >= maxIter || converged()) { tunerEndSearch(); return; }
    tphase = 0;
  }
}
void prepare() {                          // fixe P pour l'essai (ti, tphase)
  while (tstate == TS_RUN) {
    memcpy(P, bestP, sizeof P);
    if (tphase == 0) return;
    if (!tuneSel[ti]) { nextGain(); continue; }
    float c = bestP[ti] + (tphase == 1 ? DP[ti] : -DP[ti]);
    if (allowed(ti, c)) { P[ti] = c; return; }
    if (tphase == 1) { tphase = 2; continue; }
    DP[ti] *= 0.9f; nextGain();
  }
}
bool tunerStart() {
  bool any = false;
  for (int i = 0; i < NG; i++) any |= tuneSel[i];
  if (!any || !gainsOkForTune()) return false;
  for (int i = 0; i < NG; i++) {
    startP[i] = bestP[i] = P[i] = G[i];
    DP[i] = max(stepFrac * fabsf(G[i]), stepMin[i]);
  }
  bestJ = 1e9f; ti = 0; tphase = 0; iter = 0; logCount = 0;
  tstate = TS_RUN; tuning = true;
  applyP(P); beginTrial();
  return true;
}
void tunerResult(float J, bool fell) {   // résultat d'un essai de recherche
  logTrial(J, fell);
  if (tphase == 0)                       { bestJ = J; ti = 0; tphase = 1; }
  else if (J < bestJ * (1 - minImpr))    { bestJ = J; memcpy(bestP, P, sizeof P); DP[ti] *= 1.1f; nextGain(); }
  else if (tphase == 1)                  { tphase = 2; }
  else                                   { DP[ti] *= 0.9f; nextGain(); }
  if (tstate == TS_RUN) { prepare(); if (tstate == TS_RUN) applyP(P); }
  beginTrial();
}
void validResult(float J1, float J2, bool fell) {
  bool ok = !fell && J2 <= 2.0f * max(J1, 0.05f);
  logTrial(fell ? fallJ : 0.5f * (J1 + J2), fell);
  tuning = false;
  tstate = ok ? TS_OK : TS_FAIL;
  applyP(ok ? bestP : startP);
  dirty = true;
}
void trialSample(float e, float w, float vcm, float xcm, bool sat, float dt) {
  if (!trialX0Set) { trialX0 = xcm; trialX0Set = true; }
  xcm -= trialX0;                        // x depuis le début de l'essai
  trialT = trialT + dt;
  if (trialT <= settleS) return;
  float len = trialLen();
  int h = (trialT - settleS) < 0.5f * len ? 0 : 1;
  sumC[h] += e * e + wRate * w * w + wV * vcm * vcm + wX * xcm * xcm + (sat ? wSat : 0);
  nC[h]++;
  if (sat) nSat++;
  if (trialT >= settleS + len && nC[0] > 0 && nC[1] > 0) {
    float J1 = sumC[0] / nC[0], J2 = sumC[1] / nC[1];
    if (tstate == TS_VALID) validResult(J1, J2, false);
    else tunerResult(0.5f * (J1 + J2) + wGrow * max(0.0f, J2 - J1), false);
  }
}
void trialFell() {
  if (tstate == TS_VALID) { validResult(0, 0, true); return; }
  float frac = trialT / (settleS + measureS);
  tunerResult(fallJ * (2.0f - min(frac, 1.0f)), true);
}

// ---------------- Tâche de contrôle (prioritaire, cœur 1) ----------------
void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  bool upTiming = false;
  uint32_t upSince = 0, lastSpdMs = 0;
  float accA = 0, accB = 0, vF = 0;
  int accN = 0;
  long lastEL = encLeft(), lastER = encRight();
  float lastPos = 0.5f * (lastEL + lastER), posRef = lastPos;
  int i2cErrors = 0;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(loopMs));
    if (otaActive) {                     // mise à jour du firmware : tout à l'arrêt
      tunerStop(); identRun = false; testMask = 0; mode = IDLE;
      drive(0, 0);
      continue;
    }
    uint32_t tStart = micros();
    const float dt = loopMs * 1e-3f;

    if (reqPwm) {
      reqPwm = false;
      ledcChangeFrequency(PWMA, pwmFreq, PWM_RES);
      ledcChangeFrequency(PWMB, pwmFreq, PWM_RES);
    }

    i2cErrors = updateImu() ? 0 : i2cErrors + 1;

    // Encodeurs : position et vitesse moyennes (imp, imp/s)
    if (reqEncReset) {
      reqEncReset = false; enc1 = 0; enc2 = 0;
      lastPos = posRef = 0; lastEL = lastER = 0;
    }
    long cL = encLeft(), cR = encRight();
    float pos = 0.5f * (cL + cR);
    float vRaw = (pos - lastPos) / dt;
    lastPos = pos;
    vF += (vRaw - vF) * (vTauMs > 0 ? dt / (vTauMs * 1e-3f + dt) : 1.0f);

    uint32_t nowMs = millis();
    if (nowMs - lastSpdMs >= 100) {
      float sdt = (nowMs - lastSpdMs) * 1e-3f;
      speedL = (cL - lastEL) / sdt; speedR = (cR - lastER) / sdt;
      lastEL = cL; lastER = cR; lastSpdMs = nowMs;
    }

    // Requêtes de la page
    if (reqDisarm) {
      reqDisarm = false; tunerStop();
      if (identRun) { identRun = false; identErr = 3; }
      mode = IDLE;
    }
    if (reqStop)  { reqStop = false; tunerStop(); }
    if (reqArm)   { reqArm = false; if (mode == IDLE && !identRun) mode = WAIT_UP; }
    if (reqTune)  { reqTune = false; if (!tuning && !identRun && tunerStart() && mode == IDLE) mode = WAIT_UP; }
    if (reqIdent) { reqIdent = false; if (mode == IDLE && !identRun) identStart(pos); }
    if (i2cErrors > 10 && mode != IDLE) { tunerStop(); mode = IDLE; }

    // Angle dans la convention « positif = penché vers l'avant »
    const float sgn = invAngle ? -1.0f : 1.0f;
    float th = sgn * theta, w = sgn * rate;

    // Mesure θ₀ ou calibration gyro (200 échantillons, à l'arrêt seulement)
    if ((reqT0 || reqCal) && mode != IDLE) reqT0 = reqCal = false;
    if (reqT0 || reqCal) {
      if (reqT0) accA += th; else { accA += lastGx; accB += lastGy; }
      if (++accN >= 200) {
        if (reqT0) { theta0 = accA / accN; dirty = true; }
        else { gxOff = accA / accN; gyOff = accB / accN; }
        reqT0 = reqCal = false; accA = accB = 0; accN = 0;
      }
    } else { accA = accB = 0; accN = 0; }

    float e = th - theta0, xr = pos - posRef;
    int outL = 0, outR = 0;

    switch (mode) {
      case IDLE:
        upTiming = false;
        if (identRun) { outL = outR = identTick(pos, cL, cR, dt); testMask = 0; }
        else if (testMask && (int32_t)(testUntil - nowMs) > 0) {
          int v = testDir * testPwmSet;
          if (testMask & 1) outL = v;
          if (testMask & 2) outR = v;
        } else testMask = 0;
        break;

      case WAIT_UP:
        testMask = 0;
        if (fabsf(e) < armDeg) {
          if (!upTiming) { upTiming = true; upSince = nowMs; }
          else if (nowMs - upSince >= (uint32_t)armHoldMs) {
            upTiming = false; posRef = pos; xr = 0; mode = BALANCE;
          }
        } else upTiming = false;
        break;

      case BALANCE: {
        testMask = 0;
        if (fabsf(e) > fallDeg) {
          mode = WAIT_UP; falls = falls + 1;
          if (tuning) trialFell();
          break;
        }
        float u = Kp * e + Kd * w + Kv * vF + Kx * xr;
        outL = outR = compensate(u);
        if (tuning) { float c = cmPerImp(); trialSample(e, w, vF * c, xr * c, abs(outL) >= 255, dt); }
        break;
      }
    }
    drive(outL, outR);

    telTheta = th; telRate = w; telU = outL; telV = vF; telX = xr;
    telLoopUs = micros() - tStart;
  }
}

// ---------------- Page web ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Réglages</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--txt:#e6e8ee;--mut:#8a90a0;--bad:#ff5c5c;--ok:#3ddc84;--warn:#ffb347}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);font:15px system-ui,sans-serif;padding:16px}
h1{font-size:20px;margin:0 0 4px}.sub{color:var(--mut);font-size:13px}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));margin-top:12px}
.card{background:var(--card);border-radius:12px;padding:14px}
.card h2{font-size:14px;margin:0 0 10px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em}
.row{display:flex;justify-content:space-between;align-items:center;margin:6px 0;font-variant-numeric:tabular-nums;gap:8px}
.row .h{display:block;color:var(--mut);font-size:12px}
.big{font-size:24px}
input[type=number]{width:110px;flex:none;background:#11141a;color:var(--txt);border:1px solid #2a2f3a;border-radius:6px;padding:6px}
input[type=checkbox]{width:22px;height:22px;flex:none}
input.ok{border-color:var(--ok)}input:disabled{opacity:.5}
button{background:#262b36;color:var(--txt);border:0;border-radius:8px;padding:10px 12px;font-size:14px;cursor:pointer}
.top{display:flex;gap:8px}.top button{flex:1;padding:14px;font-size:16px;font-weight:600}
.go{background:var(--ok);color:#000}.stop{background:var(--bad);color:#fff}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.bar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:10px}
#dirty{color:var(--warn);font-size:13px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--bad);margin-right:6px}.dot.on{background:var(--ok)}
canvas{width:100%;height:160px;background:#11141a;border-radius:8px}
table{width:100%;border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}
td,th{padding:4px 6px;text-align:right;border-bottom:1px solid #232733}th{color:var(--mut);font-weight:500}
tr.best td{color:var(--ok)}tr.fell td{color:var(--bad)}
.wide{grid-column:1/-1;overflow-x:auto}#mode{color:var(--warn)}
ol{margin:0;padding-left:20px}ol li{margin:8px 0}
.msg{font-size:13px;margin-top:8px}.msg.bad{color:var(--bad)}.msg.warn{color:var(--warn)}.msg.good{color:var(--ok)}
</style></head><body>
<h1>Robot auto-équilibré</h1>
<div class="sub"><span id="dot" class="dot"></span><span id="st">connexion…</span></div>
<div class="top" style="margin-top:12px"><button class="go" onclick="cmd('arm')">ARMER</button><button class="stop" onclick="cmd('disarm')">STOP</button></div>
<div class="bar">
<button onclick="cmd('save')">💾 Sauvegarder</button>
<button onclick="if(confirm('Remettre tous les réglages par défaut ?'))cmd('defaults')">Valeurs par défaut</button>
<button onclick="cmd('t0')">Mesurer θ₀</button>
<button onclick="cmd('cal')">Calibrer gyro</button>
<span id="dirty"></span>
</div>
<div class="grid">
<div class="card"><h2>État</h2>
  <div class="row"><span>Mode</span><span id="mode">-</span></div>
  <div class="row"><span>θ<span class="h">positif = penché vers l'avant</span></span><span class="big" id="th">-</span></div>
  <div class="row"><span>e = θ − θ₀</span><span class="big" id="e">-</span></div>
  <div class="row"><span>ω (°/s)</span><span id="w">-</span></div>
  <div class="row"><span>v roues (imp/s)</span><span id="v">-</span></div>
  <div class="row"><span>x depuis l'armement (imp)</span><span id="x">-</span></div>
  <div class="row"><span>Commande PWM</span><span id="u">-</span></div>
  <div class="row"><span>Chutes</span><span id="falls">-</span></div>
  <div class="row"><span>Calcul boucle</span><span id="lus">-</span></div>
</div>
<div class="card"><h2>Test moteurs (mode Arrêt)</h2>
  <div class="row"><span>Encodeur gauche</span><span id="el">-</span></div>
  <div class="row"><span>Vitesse gauche (imp/s)</span><span id="sl">-</span></div>
  <div class="row"><span>Encodeur droit</span><span id="er">-</span></div>
  <div class="row"><span>Vitesse droite (imp/s)</span><span id="sr">-</span></div>
  <div class="btns">
  <button onclick="test(1,1)">Gauche ▶</button><button onclick="test(1,-1)">Gauche ◀</button>
  <button onclick="test(2,1)">Droite ▶</button><button onclick="test(2,-1)">Droite ◀</button>
  <button onclick="test(3,1)">Les deux ▶</button><button onclick="test(3,-1)">Les deux ◀</button>
  <button onclick="cmd('reset')">Reset encodeurs</button>
  </div>
  <div class="sub" style="margin-top:8px">▶ = avant : la roue doit avancer et son encodeur monter. Sinon, voir la carte « Moteurs &amp; encodeurs ».</div>
</div>
<div class="card"><h2>Modèle → gains (LQR)</h2>
  <ol class="sub">
  <li>Roues en l'air, sur batterie (≈ 6 s) :<br><button onclick="cmd('ident')">Identifier moteur</button> <span id="idst"></span></li>
  <li>Reset encodeurs, 1 tour exact de la roue gauche à la main, puis<br><button onclick="cmd('setn')">N = compte gauche</button></li>
  <li>Renseigner r et l dans la carte « Modèle ».</li>
  <li><button class="go" onclick="calc()">Calculer les gains</button></li>
  </ol>
  <table id="idtab" style="margin-top:8px"></table>
  <div id="idmsg"></div>
  <div id="lqr"></div>
  <div class="btns"><button id="lqrap" style="display:none" onclick="applyLqr()">Appliquer Kp, Kd, Kv, Kx</button></div>
</div>
<div class="card"><h2>Autotune</h2>
  <div class="btns" style="margin-top:0">
  <button class="go" onclick="cmd('tune')">Lancer l'autotune</button>
  <button onclick="cmd('stop')">Arrêter (garder le meilleur)</button>
  </div>
  <div class="row" style="margin-top:10px"><span>État</span><span id="tun">-</span></div>
  <div class="row"><span>Itération</span><span id="it">-</span></div>
  <div class="row"><span>Essai en cours</span><span id="cur">-</span></div>
  <div class="row"><span>Temps d'essai</span><span id="tt">-</span></div>
  <div class="row"><span>Meilleur J</span><span id="bj">-</span></div>
  <div class="row"><span>Meilleurs gains</span><span id="bg" style="font-size:13px">-</span></div>
  <div class="sub">Part de gains qui tiennent déjà (LQR). Réglages : carte « Autotune » plus bas.</div>
</div>
<div class="card"><h2>Mise à jour du firmware</h2>
  <div class="sub">Arduino IDE : Croquis → Exporter les binaires compilés, puis choisir le fichier <b>.ino.bin</b> (pas « merged » ni « bootloader »). Le robot s'arrête et redémarre.</div>
  <div class="btns"><input type="file" id="fw" accept=".bin"><button class="go" onclick="upFw()">Téléverser</button></div>
  <div class="msg" id="fwst"></div>
</div>
<div id="params" style="display:contents"></div>
<div class="card wide"><h2>Erreur e (±15°)</h2><canvas id="cv"></canvas></div>
<div class="card wide"><h2>Essais de l'autotune</h2>
  <table><thead><tr><th>#</th><th>Itér.</th><th>Essai</th><th>Kp</th><th>Kd</th><th>Kv</th><th>Kx</th><th>J</th><th></th></tr></thead><tbody id="log"></tbody></table>
</div>
</div>
<script>
const $=id=>document.getElementById(id);
const MODES=['Arrêt','Redresse le robot…','Équilibre'],PN=['Kp','Kd','Kv','Kx'],PH=['réf.','+Δ','−Δ','validation'];
const TSN=['—','recherche en cours','validation (essai long)…','terminé ✓ gains validés','validation ratée : gains de départ remis','arrêté (meilleurs gains appliqués)'];
const IDERR=['','Vitesse nulle ou négative : règle d\'abord les sens moteurs/encodeurs (▶ doit faire monter les encodeurs).','Résultat incohérent (vitesse qui ne croît pas avec la PWM). Batterie faible ? Recommence.','Identification annulée.'];
let built=false,lastLn=-1,bestJ=1e9,tuning=0,busy=0,PV={},PL=[],lastId='',LQ=null;const E=[],N=300;
async function cmd(c){try{const t=await(await fetch('/cmd?c='+c)).text();if(t!=='ok')alert(t);loadParams();}catch(e){}}
function test(m,d){cmd('test&m='+m+'&d='+d);}
const g3=v=>String(+(+v).toPrecision(4));
const fmt=(v,t)=>t==='i'?String(Math.round(v)):String(+(+v).toPrecision(5));
async function setParam(k,v,el){
  const t=await(await fetch('/set?k='+k+'&v='+encodeURIComponent(v))).text();
  if(t!=='ok')alert(t);else if(el){el.classList.add('ok');setTimeout(()=>el.classList.remove('ok'),800);}
  await loadParams();return t==='ok';}
function build(L){
  const groups={};L.forEach(p=>(groups[p.g]=groups[p.g]||[]).push(p));
  let h='';for(const g in groups){h+=`<div class="card"><h2>${g}</h2>`;
    groups[g].forEach(p=>{const lab=`<span>${p.l}${p.h?`<span class="h">${p.h}</span>`:''}</span>`;
      h+=p.t==='b'?`<label class="row">${lab}<input type="checkbox" id="p_${p.k}"></label>`
        :`<div class="row">${lab}<input type="number" step="any" min="${p.mn}" max="${p.mx}" id="p_${p.k}"></div>`;});
    h+='</div>';}
  $('params').innerHTML=h;
  L.forEach(p=>{const el=$('p_'+p.k);
    el.addEventListener('change',()=>setParam(p.k,p.t==='b'?(el.checked?1:0):el.value,el));});
  built=true;}
function lockInputs(){PL.forEach(p=>{const el=$('p_'+p.k);if(el)el.disabled=(p.lk===1&&tuning)||(p.lk===2&&busy);});}
async function loadParams(){try{
  const L=await(await fetch('/params',{cache:'no-store'})).json();
  if(!built)build(L);PL=L;
  L.forEach(p=>{PV[p.k]=p.v;const el=$('p_'+p.k);
    if(document.activeElement===el)return;
    if(p.t==='b')el.checked=p.v>0.5;else el.value=fmt(p.v,p.t);});
  lockInputs();
}catch(e){}}

// ---- LQR ----
function mmul(A,B){const n=A.length,m=B[0].length,k=B.length,C=[];for(let i=0;i<n;i++){C.push(new Array(m).fill(0));for(let j=0;j<m;j++){let s=0;for(let t=0;t<k;t++)s+=A[i][t]*B[t][j];C[i][j]=s;}}return C;}
function tr(A){return A[0].map((_,j)=>A.map(r=>r[j]));}
function madd(A,B,s=1){return A.map((r,i)=>r.map((v,j)=>v+s*B[i][j]));}
function expm(M){let S=M.map((r,i)=>r.map((_,j)=>i===j?1:0)),T=S.map(r=>r.slice());
  for(let k=1;k<30;k++){T=mmul(T,M).map(r=>r.map(v=>v/k));S=madd(S,T);}return S;}
// États : x (m, mesuré par les encodeurs), q (m/s), θ (rad), ω (rad/s)
// Moteur : τ·q' + q = Ks·u.  Corps : (l_eff + r)·ω' = g·θ − q'.
function lqr(p){
  const g=9.81,L=p.l*p.lf+p.r,Ks=p.Km*2*Math.PI*p.r/p.N;
  const A=[[0,1,0,0],[0,-1/p.tau,0,0],[0,0,0,1],[0,1/(p.tau*L),g/L,0]];
  const B=[0,Ks/p.tau,0,-Ks/(p.tau*L)];
  const M=[...A.map((r,i)=>[...r,B[i]]),[0,0,0,0,0]].map(r=>r.map(v=>v*p.dt));
  const X=expm(M),Ad=X.slice(0,4).map(r=>r.slice(0,4)),Bd=X.slice(0,4).map(r=>[r[4]]);
  const th=p.thmax*Math.PI/180,Q=[[1/p.xmax**2,0,0,0],[0,0,0,0],[0,0,1/th**2,0],[0,0,0,0]],R=1/p.umax**2;
  let P=Q.map(r=>r.slice()),K=[0,0,0,0];const AdT=tr(Ad),BdT=tr(Bd);
  for(let it=0;it<200000;it++){
    const PA=mmul(P,Ad),PB=mmul(P,Bd),s=R+mmul(BdT,PB)[0][0],BPA=mmul(BdT,PA)[0];
    const Kn=BPA.map(v=>v/s);
    const Pn=madd(madd(Q,mmul(AdT,PA)),mmul(mmul(AdT,PB),[Kn]),-1);
    let d=0;for(let i=0;i<4;i++)for(let j=0;j<4;j++)d=Math.max(d,Math.abs(Pn[i][j]-P[i][j])/(Math.abs(P[i][j])+1e-9));
    P=Pn;K=Kn;if(d<1e-10&&it>10)break;}
  let s=[0,0,2*Math.PI/180,0],umx=0,xmx=0;
  for(let i=0;i<4/p.dt;i++){const u=-(K[0]*s[0]+K[1]*s[1]+K[2]*s[2]+K[3]*s[3]);umx=Math.max(umx,Math.abs(u));
    s=Ad.map((r,k)=>r[0]*s[0]+r[1]*s[1]+r[2]*s[2]+r[3]*s[3]+Bd[k][0]*u);xmx=Math.max(xmx,Math.abs(s[0]));}
  const ok=Math.abs(s[2])<0.2*Math.PI/180&&isFinite(umx);
  const m=2*Math.PI*p.r/p.N;
  return {Kp:-K[2]*Math.PI/180,Kd:-K[3]*Math.PI/180,Kv:-K[1]*m,Kx:-K[0]*m,umax:umx,xmax:xmx,ok};}
function calc(){
  const miss=[];
  if(!(PV.km>0&&PV.taum>0))miss.push('identifier le moteur (K_m, τ)');
  if(!(PV.nimp>0))miss.push('mesurer N');
  if(!(PV.rmm>0))miss.push('rayon de roue r');
  if(!(PV.lmm>0))miss.push('hauteur du centre de masse l');
  $('lqrap').style.display='none';LQ=null;
  if(miss.length){$('lqr').innerHTML=`<div class="msg bad">Il manque : ${miss.join(', ')}.</div>`;return;}
  const r=lqr({Km:PV.km,tau:PV.taum/1000,N:PV.nimp,r:PV.rmm/1000,l:PV.lmm/1000,lf:PV.lf,dt:PV.loopms/1000,xmax:PV.lqx/100,thmax:PV.lqt,umax:PV.lqu});
  let h=`<table><tr><th>Kp</th><th>Kd</th><th>Kv</th><th>Kx</th></tr><tr><td>${g3(r.Kp)}</td><td>${g3(r.Kd)}</td><td>${g3(r.Kv)}</td><td>${g3(r.Kx)}</td></tr></table>`;
  h+=`<div class="msg">Simulation, départ penché de 2° : PWM max ${Math.round(r.umax)}, déplacement max ${(r.xmax*100).toFixed(1)} cm.</div>`;
  if(!r.ok)h+='<div class="msg bad">Le calcul ne stabilise pas le modèle : vérifie les valeurs du modèle.</div>';
  else if(r.umax+PV.db>255)h+='<div class="msg warn">La commande saturerait (PWM > 255) : baisse « Commande max visée » ou augmente « Angle toléré », puis recalcule.</div>';
  else h+='<div class="msg good">OK. Point de départ : il restera peut-être à affiner (autotune).</div>';
  $('lqr').innerHTML=h;
  if(r.ok){LQ=r;$('lqrap').style.display='';}}
async function applyLqr(){if(!LQ)return;
  for(const [k,v] of [['kp',LQ.Kp],['kd',LQ.Kd],['kv',LQ.Kv],['kx',LQ.Kx]])
    if(!await setParam(k,v))return;
  $('lqr').innerHTML+='<div class="msg good">Gains appliqués (pense à Sauvegarder).</div>';$('lqrap').style.display='none';}

function upFw(){const f=$('fw').files[0],st=$('fwst');
  if(!f){st.textContent='Choisis d\'abord un fichier .bin';return;}
  if(!confirm('Le robot va s\'arrêter puis redémarrer. Les réglages non sauvegardés seront perdus. Continuer ?'))return;
  const fd=new FormData();fd.append('fw',f,f.name);const x=new XMLHttpRequest();
  x.upload.onprogress=e=>{if(e.lengthComputable)st.textContent='Envoi : '+Math.round(e.loaded/e.total*100)+' %';};
  x.onload=()=>{if(x.responseText==='ok'){st.className='msg good';st.textContent='OK, redémarrage… recharge la page dans ~10 s.';}
    else{st.className='msg bad';st.textContent=x.responseText;}};
  x.onerror=()=>{st.className='msg bad';st.textContent='Erreur réseau pendant l\'envoi.';};
  st.className='msg';x.open('POST','/update');x.send(fd);}

function draw(){const cv=$('cv'),cx=cv.getContext('2d'),w=cv.width=cv.clientWidth*devicePixelRatio,h=cv.height=cv.clientHeight*devicePixelRatio;
  cx.strokeStyle='#2a2f3a';cx.lineWidth=1;cx.beginPath();cx.moveTo(0,h/2);cx.lineTo(w,h/2);cx.stroke();
  cx.strokeStyle='#4da3ff';cx.lineWidth=2*devicePixelRatio;cx.beginPath();
  E.forEach((v,i)=>{const x=i*w/(N-1),y=h/2-Math.max(-15,Math.min(15,v))/15*h/2;i?cx.lineTo(x,y):cx.moveTo(x,y);});cx.stroke();}
const trialName=t=>t.ph===3?'validation':(t.ph?PN[t.p]+' '+PH[t.ph]:'réf.');
async function loadLog(){try{const L=await(await fetch('/log')).json();
  $('log').innerHTML=L.slice().reverse().map(t=>`<tr class="${t.f?'fell':(t.ph<3&&t.J<=bestJ+1e-6?'best':'')}"><td>${t.n}</td><td>${t.ph===3?'-':t.i+1}</td><td>${trialName(t)}</td>${t.g.map(v=>`<td>${g3(v)}</td>`).join('')}<td>${t.J.toFixed(2)}</td><td>${t.f?'chute':''}</td></tr>`).join('');}catch(e){}}
function showIdent(d){
  $('idst').textContent=d.idr?`palier ${d.ids+1}/3…`:(d.ide?'':(d.idv[0]?'terminé ✓':''));
  const key=JSON.stringify([d.idr,d.ide,d.idw,d.idv,d.idt]);if(key===lastId)return;lastId=key;
  $('idtab').innerHTML=d.idv.some(v=>v)?'<tr><th>PWM</th>'+d.idu.map(u=>`<th>${u}</th>`).join('')+'</tr>'
    +'<tr><td>v (imp/s)</td>'+d.idv.map(v=>`<td>${Math.round(v)}</td>`).join('')+'</tr>'
    +'<tr><td>τ (ms)</td>'+d.idt.map(v=>`<td>${Math.round(v*1000)}</td>`).join('')+'</tr>':'';
  $('idmsg').innerHTML=d.ide?`<div class="msg bad">${IDERR[d.ide]}</div>`
    :(!d.idr&&d.idw?'<div class="msg warn">Les deux roues n\'ont pas la même vitesse (> 15 % d\'écart) : frottement, câblage ou moteur différent.</div>':'');
  if(!d.idr)loadParams();}
let tickN=0;
async function tick(){
  try{const d=await(await fetch('/s',{cache:'no-store'})).json();
    $('dot').className='dot on';$('st').textContent=d.mpu?'connecté':'connecté — MPU6050 NON DÉTECTÉ';
    $('mode').textContent=d.idr?'Identification moteur…':MODES[d.mode]+(d.busy?' (mesure…)':'');
    const e=d.th-d.t0;$('th').textContent=d.th.toFixed(1)+'°';$('e').textContent=e.toFixed(2)+'°';
    $('w').textContent=d.w.toFixed(1);$('v').textContent=d.v.toFixed(0);$('x').textContent=d.x.toFixed(0);
    $('u').textContent=d.u;$('falls').textContent=d.falls;$('lus').textContent=d.lus+' µs';
    $('el').textContent=d.el;$('er').textContent=d.er;$('sl').textContent=d.sl.toFixed(0);$('sr').textContent=d.sr.toFixed(0);
    $('dirty').textContent=d.dirty?'● modifications non sauvegardées':'';
    const b=(d.mode!==0||d.idr)?1:0;
    if(d.tun!==tuning||b!==busy){tuning=d.tun;busy=b;loadParams();}
    showIdent(d);
    $('tun').textContent=TSN[d.ts];
    $('it').textContent=d.ts===1?(d.it+1)+' / '+d.mi:'-';
    $('cur').textContent=d.ts===1?(d.ph?PN[d.ti]+' '+PH[d.ph]:'référence'):(d.ts===2?'gains finaux':'-');
    $('tt').textContent=d.tun?d.tt.toFixed(1)+' / '+d.tl.toFixed(1)+' s':'-';
    bestJ=d.bj;$('bj').textContent=d.ln?d.bj.toFixed(2):'-';
    $('bg').textContent=d.ln?d.bk.map((v,i)=>PN[i]+' '+g3(v)).join(' · '):'-';
    E.push(e);if(E.length>N)E.shift();draw();
    if(d.ln!==lastLn){lastLn=d.ln;loadLog();}
    if(d.tun&&++tickN%10===0)loadParams();
  }catch(err){$('dot').className='dot';$('st').textContent='déconnecté';}
  setTimeout(tick,150);}
loadParams();tick();
</script></body></html>
)rawliteral";

// ---------------- Handlers HTTP ----------------
bool robotIdle() { return mode == IDLE && !identRun; }

void handleParams() {
  String s;
  s.reserve(12000);
  s = "[";
  char b[96];
  for (int i = 0; i < NPARAM; i++) {
    const Param &p = PARAMS[i];
    if (i) s += ",";
    s += "{\"k\":\""; s += p.key;
    s += "\",\"l\":\""; s += p.label;
    s += "\",\"g\":\""; s += p.group;
    s += "\",\"h\":\""; s += p.hint;
    s += "\",\"t\":\""; s += (p.t == T_F ? "f" : p.t == T_I ? "i" : "b");
    snprintf(b, sizeof b, "\",\"v\":%.7g,\"mn\":%.7g,\"mx\":%.7g,\"lk\":%d}", getP(p), p.mn, p.mx, (int)p.lock);
    s += b;
  }
  s += "]";
  server.send(200, "application/json", s);
}

void handleSet() {
  Param *p = findP(server.arg("k").c_str());
  if (!p || !server.hasArg("v")) { server.send(200, "text/plain", "Paramètre inconnu."); return; }
  if (p->lock == L_TUNE && tuning) { server.send(200, "text/plain", "Autotune en cours : arrête-le avant de changer ce réglage."); return; }
  if (p->lock == L_IDLE && !robotIdle()) { server.send(200, "text/plain", "Fais STOP avant de changer ce réglage (il modifie le sens des capteurs ou des moteurs)."); return; }
  float old = getP(*p);
  setP(*p, server.arg("v").toFloat());
  if (p->p == &pwmFreq) reqPwm = true;
  if (p->p == &invAngle && getP(*p) != old) theta0 = -theta0;   // θ₀ suit la convention
  dirty = true;
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  char b[1200];
  snprintf(b, sizeof b,
    "{\"el\":%ld,\"er\":%ld,\"sl\":%.1f,\"sr\":%.1f,\"v\":%.1f,\"x\":%.1f,"
    "\"mode\":%d,\"tun\":%d,\"ts\":%d,\"mpu\":%d,\"th\":%.2f,\"t0\":%.2f,\"w\":%.1f,\"u\":%d,"
    "\"it\":%d,\"mi\":%d,\"ti\":%d,\"ph\":%d,\"tt\":%.2f,\"tl\":%.2f,\"bj\":%.3f,"
    "\"bk\":[%.5g,%.5g,%.5g,%.5g],\"ln\":%d,\"falls\":%u,\"lus\":%u,"
    "\"dirty\":%d,\"busy\":%d,"
    "\"idr\":%d,\"ids\":%d,\"ide\":%d,\"idw\":%d,"
    "\"idu\":[%d,%d,%d],\"idv\":[%.1f,%.1f,%.1f],\"idt\":[%.4f,%.4f,%.4f]}",
    encLeft(), encRight(), (float)speedL, (float)speedR, (float)telV, (float)telX,
    (int)mode, tuning ? 1 : 0, (int)tstate, mpuOk ? 1 : 0, (float)telTheta, theta0, (float)telRate, (int)telU,
    (int)iter, maxIter, (int)ti, (int)tphase, (float)trialT, settleS + trialLen(), (float)bestJ,
    bestP[0], bestP[1], bestP[2], bestP[3], (int)logCount, (unsigned)falls, (unsigned)telLoopUs,
    dirty ? 1 : 0, (reqT0 || reqCal) ? 1 : 0,
    identRun ? 1 : 0, (int)identStep, (int)identErr, identWarn ? 1 : 0,
    idU[0], idU[1], idU[2], idV[0], idV[1], idV[2], idTau[0], idTau[1], idTau[2]);
  server.send(200, "application/json", b);
}

void handleLog() {
  int count = logCount;
  int start = count > LOG_N ? count - LOG_N : 0;
  String s;
  s.reserve(4800);
  s = "[";
  char b[180];
  for (int n = start; n < count; n++) {
    const Trial &t = tlog[n % LOG_N];
    snprintf(b, sizeof b, "%s{\"n\":%d,\"i\":%d,\"p\":%d,\"ph\":%d,\"g\":[%.5g,%.5g,%.5g,%.5g],\"J\":%.3f,\"f\":%d}",
             n > start ? "," : "", n + 1, (int)t.iter, (int)t.param, (int)t.phase,
             t.g[0], t.g[1], t.g[2], t.g[3], t.J, t.fell ? 1 : 0);
    s += b;
  }
  s += "]";
  server.send(200, "application/json", s);
}

void handleCmd() {
  String c = server.arg("c");
  const char *needStop = "Fais STOP d'abord.";
  bool idle = robotIdle();
  if      (c == "arm")    reqArm = true;
  else if (c == "disarm") reqDisarm = true;
  else if (c == "stop")   reqStop = true;
  else if (c == "reset")  reqEncReset = true;
  else if (c == "tune") {
    bool any = false;
    for (int i = 0; i < NG; i++) any |= tuneSel[i];
    if (!any) { server.send(200, "text/plain", "Coche au moins un gain à régler (carte Autotune)."); return; }
    if (!gainsOkForTune()) { server.send(200, "text/plain", "Kv et Kx doivent être > 0 avant l'autotune : un PD seul ne peut pas tenir le robot. Applique d'abord les gains LQR et vérifie qu'il tient."); return; }
    reqTune = true;
  }
  else if (c == "test") {
    if (!idle) { server.send(200, "text/plain", needStop); return; }
    testDir   = server.arg("d").toInt() < 0 ? -1 : 1;
    testUntil = millis() + testMs;
    testMask  = constrain(server.arg("m").toInt(), 0, 3);
  }
  else if (c == "ident") {
    if (!idle) { server.send(200, "text/plain", needStop); return; }
    reqIdent = true;
  }
  else if (c == "setn") {
    long n = labs(encLeft());
    if (n < 10) { server.send(200, "text/plain", "Compte gauche trop faible : Reset encodeurs, puis fais exactement 1 tour de la roue gauche à la main."); return; }
    nImp = n; dirty = true;
  }
  else if (c == "t0" || c == "cal") {
    if (!idle) { server.send(200, "text/plain", needStop); return; }
    if (c == "t0") reqT0 = true; else reqCal = true;
  }
  else if (c == "save") {
    if (!idle) { server.send(200, "text/plain", "Fais STOP avant de sauvegarder."); return; }
    saveParams();
  }
  else if (c == "defaults") {
    if (!idle || tuning) { server.send(200, "text/plain", needStop); return; }
    prefs.clear();
    for (int i = 0; i < NPARAM; i++) setP(PARAMS[i], PARAMS[i].def);
    for (int i = 0; i < NG; i++) bestP[i] = G[i];
    reqPwm = true; dirty = false;
  }
  else { server.send(400, "text/plain", "commande inconnue"); return; }
  server.send(200, "text/plain", "ok");
}

// ---------------- Mise à jour par WiFi ----------------
void otaStopMotors() { otaActive = true; digitalWrite(STBY, LOW); }   // + coupure matérielle du TB6612
void otaResume()     { otaActive = false; digitalWrite(STBY, HIGH); }

void handleUpdateDone() {                // téléversement depuis la page web
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", ok ? "ok" : "Échec de la mise à jour (fichier .bin invalide ou trop gros ?).");
  if (ok) { delay(500); ESP.restart(); }
  else otaResume();
}
void handleUpdateUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    otaStopMotors();
    Serial.printf("Mise à jour web : %s\n", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("Mise à jour OK : %u octets\n", (unsigned)up.totalSize);
    else Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaResume();
  }
}
void setupArduinoOta() {                 // téléversement depuis l'Arduino IDE
  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([] { otaStopMotors(); Serial.println("OTA : début"); });
  ArduinoOTA.onEnd([] { Serial.println("OTA : terminé, redémarrage"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA : erreur %u\n", e); otaResume(); });
  ArduinoOTA.begin();
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);

  prefs.begin("robot", false);
  loadParams();
  for (int i = 0; i < NG; i++) bestP[i] = G[i];

  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, pwmFreq, PWM_RES);
  ledcAttach(PWMB, pwmFreq, PWM_RES);
  drive(0, 0);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  st1 = readPins(ENC1_A, ENC1_B);
  st2 = readPins(ENC2_A, ENC2_B);
  attachInterrupt(ENC1_A, isr1, CHANGE); attachInterrupt(ENC1_B, isr1, CHANGE);
  attachInterrupt(ENC2_A, isr2, CHANGE); attachInterrupt(ENC2_B, isr2, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  mpuOk = mpuInit();
  Serial.printf("MPU6050 : %s\n", mpuOk ? "OK" : "ERREUR");
  if (mpuOk) calibrateBlocking();   // robot immobile !

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("WiFi : %s  ->  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  server.on("/", [] { server.send_P(200, "text/html", PAGE); });
  server.on("/s", handleStatus);
  server.on("/params", handleParams);
  server.on("/set", handleSet);
  server.on("/log", handleLog);
  server.on("/cmd", handleCmd);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.begin();
  setupArduinoOta();

  digitalWrite(STBY, HIGH);
  xTaskCreatePinnedToCore(controlTask, "ctrl", 6144, nullptr, 5, nullptr, 1);
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  delay(1);
}
