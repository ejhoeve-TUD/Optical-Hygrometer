// Pins
const int refPin = 9;      
const int signalPin = A0;  

// Timing and Frequency configuration (100 Hz)
const unsigned long halfPeriod = 5000; 
unsigned long lastToggleTime = 0;
bool refState = LOW;

// Rolling Average Configuration
// 400 half-cycles = 200 full cycles. 
// Reduce this number if your Arduino runs out of memory (RAM).
const int FILTER_SIZE = 400; 
int history[FILTER_SIZE];     // Circular buffer to hold past samples
int historyIndex = 0;         // Tracks the current position in the buffer
long runningSum = 0;          // Keeps track of the total sum of the buffer

// DC offset tracking
const int dcOffset = 512; 

void setup() {
  pinMode(refPin, OUTPUT);
  Serial.begin(115200);
  
  // Initialize buffer to 0
  for (int i = 0; i < FILTER_SIZE; i++) {
    history[i] = 0;
  }
  
  lastToggleTime = micros();
}

void loop() {
  unsigned long currentTime = micros();

  // 1. Generate the Square Wave Reference
  if (currentTime - lastToggleTime >= halfPeriod) {
    lastToggleTime += halfPeriod;
    refState = !refState;
    digitalWrite(refPin, refState);
    
    // 2. Sample and remove DC bias
    int rawSignal = analogRead(signalPin);
    int acSignal = rawSignal - dcOffset; 

    // 3. Demodulation (Mixer)
    int currentSample = 0;
    if (refState == HIGH) {
      currentSample = acSignal;   // Multiply by +1
    } else {
      currentSample = -acSignal;  // Multiply by -1
    }

    // 4. Rolling Average Calculation (The Fast Way)
    // Subtract the oldest sample about to be overwritten from the running sum
    runningSum -= history[historyIndex];
    
    // Put the new sample into the buffer
    history[historyIndex] = currentSample;
    
    // Add the new sample to the running sum
    runningSum += currentSample;
    
    // Move the index forward, wrapping around to 0 if we hit the end
    historyIndex++;
    if (historyIndex >= FILTER_SIZE) {
      historyIndex = 0;
    }

    // 5. High-Frequency Output
    // Calculate magnitude. We divide by FILTER_SIZE / 2 to match the scale of the old code
    long magnitude = abs(runningSum) / (FILTER_SIZE / 2);

    // This will now stream smoothly to the Serial Plotter at 200 Hz
    Serial.print("Magnitude:");
    Serial.println(magnitude);
  }
}