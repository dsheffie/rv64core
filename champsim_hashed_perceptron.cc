#include <cstdint>
#include <iostream>
#include <array>
#include <set>

/*

  This code implements a hashed perceptron branch predictor using geometric
  history lengths and dynamic threshold setting.  It was written by Daniel
  A. Jiménez in March 2019.


  The original perceptron branch predictor is from Jiménez and Lin, "Dynamic
  Branch Prediction with Perceptrons," HPCA 2001.

  The idea of using multiple independently indexed tables of perceptron weights
  is from Jiménez, "Fast Path-Based Neural Branch Prediction," MICRO 2003 and
  later expanded in "Piecewise Linear Branch Prediction" from ISCA 2005.

  The idea of using hashes of branch history to reduce the number of independent
  tables is documented in three contemporaneous papers:

  1. Seznec, "Revisiting the Perceptron Predictor," IRISA technical report, 2004.

  2. Tarjan and Skadron, "Revisiting the Perceptron Predictor Again," UVA
  technical report, 2004, expanded and published in ACM TACO 2005 as "Merging
  path and gshare indexing in perceptron branch prediction"; introduces the term
  "hashed perceptron."

  3. Loh and Jiménez, "Reducing the Power and Complexity of Path-Based Neural
  Branch Prediction," WCED 2005.

  The ideas of using "geometric history lengths" i.e. hashing into tables with
  histories of exponentially increasing length, as well as dynamically adjusting
  the theta parameter, are from Seznec, "The O-GEHL Branch Predictor," from CBP
  2004, expanded later as "Analysis of the O-GEometric History Length Branch
  Predictor" in ISCA 2005.

  This code uses these ideas, but prefers simplicity over absolute accuracy (I
  wrote it in about an hour and later spent more time on this comment block than
  I did on the code). These papers and subsequent papers by Jiménez and other
  authors significantly improve the accuracy of perceptron-based predictors but
  involve tricks and analysis beyond the needs of a tool like ChampSim that
  targets cache optimizations. If you want accuracy at any cost, see the winners
  of the latest branch prediction contest, CBP 2016 as of this writing, but
  prepare to have your face melted off by the complexity of the code you find
  there. If you are a student being asked to code a good branch predictor for
  your computer architecture class, don't copy this code; there are much better
  sources for you to plagiarize.
*/




// speed for dynamic threshold setting
#define SPEED 18

// 12-bit indices for the tables

#define LOG_TABLE_SIZE 12
#define TABLE_SIZE (1 << LOG_TABLE_SIZE)

// this many 12-bit words will be kept in the global history

#define MAXHIST 232

#define NGHIST_WORDS (MAXHIST / LOG_TABLE_SIZE + 1)

static int history_lengths[] = {0, 3, 4, 6, 8, 10, 14, 19, 26, 36, 49, 67, 91, 125, 170,  MAXHIST};

#define NTABLES ((sizeof(history_lengths)/sizeof(history_lengths[0])))

// tables of 8-bit weights

#define NUM_CPUS 1

static int tables[NTABLES][TABLE_SIZE] = {0};

// words that store the global history

static std::array<uint32_t,NGHIST_WORDS> ghist_words = {0};
static std::array<uint32_t,NGHIST_WORDS> speculative_ghist_words = {0};

// remember the indices into the tables from prediction to update

static uint64_t indices[1<<16][NTABLES] = {0};

// initialize theta to something reasonable,
static int theta = {10};

// initialize counter for threshold setting algorithm
static int tc = {0};

static std::array<uint64_t, 1<<16> pcs = {~0UL};

// perceptron sum
static std::array<int, 1<<16> yout = {0};

static std::set<long long> branches;

extern "C" {

  int hashed_perceptron_predict(long long pc, int key) {
    // initialize perceptron sum
    //std::cout << "making prediction for key " << key << "\n";

    if(branches.find(pc) == branches.end()) {
      return 0;
    }
    
    yout.at(key) = 0;
    pcs.at(key) = pc;


    
    //std::cout << std::hex << "pc = " << std::hex << pc << std::dec
    //<<  ", key = " << key << "\n";
  
      
    // for each table...
    for (int i = 0; i < NTABLES; i++) {
      // n is the history length for this table
      int n = history_lengths[i];
      // hash global history bits 0..n-1 into x by XORing the words from the
      // ghist_words array

      uint64_t x = 0;

      // most of the words are 12 bits long

      int most_words = n / LOG_TABLE_SIZE;

      // the last word is fewer than 12 bits

      int last_word = n % LOG_TABLE_SIZE;

      // XOR up to the next-to-the-last word

      int j;
      for (j = 0; j < most_words; j++)
	x ^= ::speculative_ghist_words[j];

      // XOR in the last word

      x ^= ::speculative_ghist_words[j] & ((1 << last_word) - 1);

      // XOR in the PC to spread accesses around (like gshare)

      x ^= pc;

      // stay within the table size

      x &= TABLE_SIZE - 1;

      // remember this index for update

      indices[key][i] = x;

      // add the selected weight to the perceptron sum

      yout[key] += ::tables[i][x];
    }
    bool b = yout[key] >= 1;
    for (int i = 0; i < NGHIST_WORDS; i++) {

      // shift b into the lsb of the current word

      ::speculative_ghist_words[i] <<= 1;
      ::speculative_ghist_words[i] |= b;

      // get b as the previous msb of the current word

      b = !!(::speculative_ghist_words[i] & TABLE_SIZE);
      ::speculative_ghist_words[i] &= TABLE_SIZE - 1;
    }
    std::cout << std::hex << pc << std::dec << ",key = " << key << ",pred = " << (yout[key]>=1) << "\n";
    return yout[key] >= 1;
  }

  void hashed_perceptron_update(int key, long long pc, int taken) {

    if(branches.find(pc) == branches.end()) {
      //printf("learning branch at pc %llx\n", pc);
      branches.insert(pc);
      return;
    }
    
    // was this prediction correct?
    bool correct = taken == (::yout.at(key) >= 1);

    if(pcs.at(key) != pc) {
      std::cout << "perceptron update with key " << key << ", pc mismatch\n";
      std::cout << "retirement pc " << std::hex << pc << std::dec << "\n";
      std::cout << "table pc      " << std::hex << pcs[key] << std::dec << "\n";
      //exit(-1);
    }

    // insert this branch outcome into the global history

    bool b = taken;
    for (int i = 0; i < NGHIST_WORDS; i++) {

      // shift b into the lsb of the current word

      ::ghist_words[i] <<= 1;
      ::ghist_words[i] |= b;

      // get b as the previous msb of the current word

      b = !!(::ghist_words[i] & TABLE_SIZE);
      ::ghist_words[i] &= TABLE_SIZE - 1;
    }
    
    if(not(correct)) {
      std::cout << "mispredict, need to copy arch hist to speculative hist for pc " << std::hex << pc << std::dec << "\n";
      speculative_ghist_words = ghist_words;
    }
    
    // get the magnitude of yout

    int a = (::yout[key] < 0) ? -::yout[key] : ::yout[key];

    // perceptron learning rule: train if misprediction or weak correct prediction

    if (!correct || a < ::theta) {
      // update weights
      for (int i = 0; i < NTABLES; i++) {
	// which weight did we use to compute yout?

	int* c = &::tables[i][::indices[key][i]];

	// increment if taken, decrement if not, saturating at 127/-128

	if (taken) {
	  if (*c < 127)
	    (*c)++;
	} else {
	  if (*c > -128)
	    (*c)--;
	}
      }

      // dynamic threshold setting from Seznec's O-GEHL paper

      if (!correct) {

	// increase theta after enough mispredictions

	::tc++;
	if (::tc >= SPEED) {
	  ::theta++;
	  ::tc = 0;
	}
      } else if (a < ::theta) {

	// decrease theta after enough weak but correct predictions

	::tc--;
	if (::tc <= -SPEED) {
	  ::theta--;
	  ::tc = 0;
	}
      }
    }

    pcs.at(key) = ~(0);
    //return correct;
  }
}

