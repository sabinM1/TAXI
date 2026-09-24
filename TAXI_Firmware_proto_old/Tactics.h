/**
 * Tactics.h — Opening Strategy Definitions
 * 
 * 8 DIP-switch-selectable opening tactics.
 * Each tactic is a timed sequence of motor commands executed immediately
 * after the START-STOP signal.
 * 
 * DIP switch mapping (D5=LSD, D6, D7=MSB):
 *   000 (all UP)   → Strategy 0: Immediate forward rush
 *   001 (D5 DOWN)  → Strategy 1: 90° right turn → forward rush
 *   010 (D6 DOWN)  → Strategy 2: 90° left turn → forward rush
 *   011 (D5+D6 DN) → Strategy 3: Zigzag right-to-left cross → search
 *   100 (D7 DOWN)  → Strategy 4: Zigzag left-to-right cross → search
 *   101 (D7+D5 DN) → Strategy 5: 180° spin → forward (rear-position starts)
 *   110 (D7+D6 DN) → Strategy 6: Wait-200ms-then-rush (deceptive)
 *   111 (all DOWN) → Strategy 7: Curve-to-right sweep (aggressive search)
 * 
 * NOTE: DIP pins have internal pull-up. Switch DOWN = reads 1.
 * 
 * STRATEGY FILE FORMAT (for HTML GUI):
 * Each strategy is an array of {action, leftSpeed, rightSpeed, duration_ms} steps.
 * The HTML strategy editor will export strategies.h with the user's definitions.
 * 
 * WHY: User Q5 wants multiple strategies, some reactive (Option A — wait for
 * opponent to move), some active search (Option B — sweep the dohyo).
 * User Q11 wants the HTML GUI to program these visually and export them.
 */

#ifndef TACTICS_H
#define TACTICS_H

#include <Arduino.h>
#include "Config.h"
#include "Motors.h"

// ============================================================================
// TACTIC STRUCTURE
// ============================================================================

// A single step in a tactic sequence
struct TacticStep {
    int8_t  leftSpeed;      // -100 to 100 (negative = reverse)
    int8_t  rightSpeed;     // -100 to 100
    uint16_t durationMs;    // How long to hold this command (ms)
};

// A complete tactic (sequence of steps, terminated by durationMs=0)
struct Tactic {
    const char* name;
    const char* description;
    TacticStep* steps;
    uint8_t    stepCount;
};

// ============================================================================
// BUILT-IN DEFAULT TACTICS
// ============================================================================

/**
 * Strategy 0: Immediate forward rush (REACTIVE — Option A)
 * Drives straight forward at full speed immediately.
 * Best when starting facing the opponent.
 */
static TacticStep tactic0_steps[] = {
    { 100, 100, 1000 },  // Full forward for 1 second
    { 0, 0, 0 }          // Terminator
};
static Tactic tactic0 = {"Rush Forward", "Immediate full-speed charge", tactic0_steps, 1};

/**
 * Strategy 1: 90° right turn → forward rush
 * Turns right first (useful if started at angle), then charges.
 */
static TacticStep tactic1_steps[] = {
    { 100, -100, 105 },  // Pivot right
    { -100, 100, 40 },   // Counter-pivot (brake)
    { 100, 100, 1000 },  // Forward rush
    { 0, 0, 0 }
};
static Tactic tactic1 = {"Right Rush", "90° right turn then charge", tactic1_steps, 3};

/**
 * Strategy 2: 90° left turn → forward rush
 * Turns left first, then charges.
 */
static TacticStep tactic2_steps[] = {
    { -100, 100, 105 },  // Pivot left
    { 100, -100, 40 },   // Counter-pivot
    { 100, 100, 1000 },  // Forward rush
    { 0, 0, 0 }
};
static Tactic tactic2 = {"Left Rush", "90° left turn then charge", tactic2_steps, 3};

/**
 * Strategy 3: Zigzag right-to-left cross (ACTIVE SEARCH — Option B)
 * Sweeps across the dohyo in a zigzag pattern, covering center area.
 */
static TacticStep tactic3_steps[] = {
    {  100, -100, 70 },   // Pivot right
    { -100,  100, 30 },   // Counter
    {  100,  100, 100 },  // Forward burst
    { -100, -100, 30 },   // Back
    { -100,  100, 120 },  // Pivot left
    {  100, -100, 30 },   // Counter
    {  100,  100, 200 },  // Forward longer
    { -100, -100, 30 },   // Back
    { 0, 0, 0 }
};
static Tactic tactic3 = {"Zigzag R→L", "Right-to-left cross sweep", tactic3_steps, 8};

/**
 * Strategy 4: Zigzag left-to-right cross (ACTIVE SEARCH — Option B)
 * Mirror of strategy 3.
 */
static TacticStep tactic4_steps[] = {
    { -100,  100, 70 },   // Pivot left
    {  100, -100, 30 },   // Counter
    {  100,  100, 100 },  // Forward
    { -100, -100, 30 },   // Back
    {  100, -100, 120 },  // Pivot right
    { -100,  100, 30 },   // Counter
    {  100,  100, 200 },  // Forward
    { -100, -100, 30 },   // Back
    { 0, 0, 0 }
};
static Tactic tactic4 = {"Zigzag L→R", "Left-to-right cross sweep", tactic4_steps, 8};

/**
 * Strategy 5: 180° spin → forward
 * For rear-position starts: spins to face center, then charges.
 */
static TacticStep tactic5_steps[] = {
    { 100, -100, 180 },  // 180° pivot right
    { -100, 100, 40 },   // Counter
    { 100, 100, 1000 },  // Rush
    { 0, 0, 0 }
};
static Tactic tactic5 = {"180° Spin Rush", "Spin to face center, then charge", tactic5_steps, 3};

/**
 * Strategy 6: Wait-200ms-then-rush (DECEPTIVE — Option A variant)
 * Holds position briefly, then charges. Deceptive timing against opponents
 * that predict immediate rushes.
 */
static TacticStep tactic6_steps[] = {
    { 0, 0, 200 },       // Wait (deceptive)
    { 100, 100, 1000 },  // Rush
    { 0, 0, 0 }
};
static Tactic tactic6 = {"Delayed Rush", "Wait 200ms then charge", tactic6_steps, 2};

/**
 * Strategy 7: Curve-to-right sweep (AGGRESSIVE SEARCH — Option B)
 * Enters a rightward-curving sweep that covers the dohyo.
 * Good for unknown opponent positioning.
 */
static TacticStep tactic7_steps[] = {
    { 100, 30, 500 },   // Wide arc right
    { 100, 100, 300 },  // Forward burst
    { 30, 100, 500 },   // Wide arc left
    { 100, 100, 300 },  // Forward
    { 0, 0, 0 }
};
static Tactic tactic7 = {"Sweep Right", "Curve-to-right aggressive search", tactic7_steps, 4};

// ============================================================================
// TACTIC REGISTRY
// ============================================================================

// Array of all 8 tactics, indexed by DIP switch value
static Tactic* tacticRegistry[STRATEGY_COUNT] = {
    &tactic0, &tactic1, &tactic2, &tactic3,
    &tactic4, &tactic5, &tactic6, &tactic7
};

// ============================================================================
// TACTIC EXECUTION ENGINE
// ============================================================================

// Current tactic execution state
static uint8_t  tacticCurrentIndex = 0;       // Which tactic is active
static uint8_t  tacticCurrentStep = 0;        // Which step we're on
static uint32_t tacticStepStart = 0;          // When current step started
static bool     tacticInitialized = false;    // Has tactic been started?

/**
 * Read DIP switches and return the selected strategy index (0–7).
 * DIP: D5=LSB, D6, D7=MSB. DOWN position = logic 1.
 */
uint8_t tacticReadDIP() {
    uint8_t index = 0;
    if (digitalRead(DIP1_PIN) == HIGH) index |= 0x01;  // D5 DOWN
    if (digitalRead(DIP2_PIN) == HIGH) index |= 0x02;  // D6 DOWN
    if (digitalRead(DIP3_PIN) == HIGH) index |= 0x04;  // D7 DOWN
    return index;
}

/**
 * Initialize the tactic engine. Called when entering STATE_TACTIC.
 * Reads DIP switches once and locks in the strategy.
 */
void tacticInit() {
    tacticCurrentIndex = tacticReadDIP();
    tacticCurrentStep = 0;
    tacticStepStart = millis();
    tacticInitialized = true;
    
    #if SERIAL_DEBUG
    Serial.print(F("[Tactic] Strategy "));
    Serial.print(tacticCurrentIndex);
    Serial.print(F(": "));
    Serial.println(tacticRegistry[tacticCurrentIndex]->name);
    #endif
}

/**
 * Execute one step of the current tactic.
 * Called every main loop iteration while in STATE_TACTIC.
 * 
 * @return true when tactic sequence is complete
 */
bool tacticExecute() {
    if (!tacticInitialized) {
        tacticInit();
    }
    
    Tactic* tactic = tacticRegistry[tacticCurrentIndex];
    
    // Check if we've reached the end of the sequence
    if (tacticCurrentStep >= tactic->stepCount) {
        tacticInitialized = false;
        return true;  // Tactic complete
    }
    
    TacticStep* step = &tactic->steps[tacticCurrentStep];
    
    // Apply motor command (non-blocking — sets PWM and returns)
    motorsSet(step->leftSpeed, step->rightSpeed);
    
    // Check if step duration has elapsed
    if (millis() - tacticStepStart >= step->durationMs) {
        tacticCurrentStep++;
        tacticStepStart = millis();
    }
    
    return false;  // Still executing
}

/**
 * Initialize DIP switch pins with pull-ups.
 * Called once from setup().
 */
void tacticPinsInit() {
    pinMode(DIP1_PIN, INPUT);
    pinMode(DIP2_PIN, INPUT);
    pinMode(DIP3_PIN, INPUT);
    digitalWrite(DIP1_PIN, HIGH);  // Enable internal pull-up
    digitalWrite(DIP2_PIN, HIGH);
    digitalWrite(DIP3_PIN, HIGH);
    
    #if SERIAL_DEBUG
    Serial.print(F("[Tactic] DIP switches on D5,D6,D7. Current: "));
    Serial.println(tacticReadDIP());
    #endif
}

#endif // TACTICS_H
