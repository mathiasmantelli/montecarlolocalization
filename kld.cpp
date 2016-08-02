//#include "kld.h"

//kld::kld()
//{

//}

#include "kld.h"
#include <fstream>
#include <iostream>
#include <math.h>

vector<float> kld_sampling::ztable;

// Constructs Z-table (from ztable.data) to lookup statistics.
kld_sampling::kld_sampling() {
  if (kld_sampling::ztable.empty())
    build_table();
}


/**
   Initialize a round of KLD sampling.  Takes in kld-parameters:
   quantile, kld-error, bin size, minimum number of samples.

   Some extra informations:
   -quantile: Probability that the KL-distance between the discrete, sampled distribution (set by bin-size) is
              less than ERROR from the true distribution. Must be between 0.5 and 1.0.

   -kld-error: Target KL-distance between discrete, sampled distribution and true underlying distribution. Must
               be greater than 0.

   -bin size: KLD-sampling uses discrete bins to detect how well the underlying distribution is being sampled.
              The larger the bins, the fewer particles needed, but the less accurate the discrete, sampled
              distribution will be. Value must be greater than 0.

   -minimum number of samples: Minimum number of samples. KLD-sampling always samples at least 10 samples.
**/

void kld_sampling::init(float quantile, float err, const vector<float>& bsz, int sample_min) {
  support_samples=0;
  num_samples=0;
  if (sample_min < absolute_min)
    kld_samples=absolute_min;
  else kld_samples=sample_min;

  bins.clear();

  confidence=quantile-0.5; // ztable is from right side of mean
  confidence=fmin(0.49998,fmax(0,confidence));

  max_error=err;
  bin_size=bsz;

  zvalue=4.1; //    4.1, why?
  for (unsigned int i=0; i<kld_sampling::ztable.size();i++)
    if (kld_sampling::ztable[i] >= confidence) {
      zvalue=i/100.0; //    i/100.00, why?
      break;
    }
}


/**
   Update kld-sampler with the last sample drawn.  Returns a guess at
   the number of samples needed before the distribution (which is
   unknown) is adequately sampled.
**/
int kld_sampling::update(const vector<float>& sample) {
  if (bin_size.empty()) {
    cerr << "kld-sampling.cc: Must run init() before update()\n";
    exit (-1);
  }

  if (sample.size() != bin_size.size()){
    cerr << "kld-sampling.cc: Sample size not the same number of dimensions as the bins\n";
    exit(-1);
  }

  curr_sample=sample;
  num_samples++;

  if (in_empty_bin()) {
    support_samples++;
    if (support_samples >=2) {
      int k=support_samples-1;
      k=(int)ceil(k/(2*max_error)*pow(1-2/(9.0*k)+sqrt(2/(9.0*k))*zvalue,3));
      if (k > kld_samples)
        kld_samples=k;
    }
  }
  return kld_samples;
}


/**
   Builds a z-table which is necessary for the statiscal kld-sampling.
**/
void kld_sampling::build_table() {
  float tmp;
  ifstream ifile("ztable.data");

  if (ifile.is_open()) {
    while (!ifile.eof()) {
      ifile >> tmp;
      kld_sampling::ztable.push_back(tmp);
    }
  }
  else {
    cerr << "kld-sampling.cc: ztable.data does not exist. Error!\n";
    exit(-1);
  }

}


/**
   Determines whether a sample falls into a bin that has already been
   sampled.
 **/
bool kld_sampling::in_empty_bin() {

  vector<float> curr_bin;

  for (unsigned int i=0; i<curr_sample.size(); i++)
    curr_bin.push_back(floor(curr_sample[i]/bin_size[i]));

  for (unsigned int i=0; i < bins.size(); i++)
    if (curr_bin==bins[i]) {
      return false;
    }
  bins.push_back(curr_bin);
  return true;
}
