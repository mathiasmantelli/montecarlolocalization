//#ifndef KLD_H
//#define KLD_H


//class kld
//{
//public:
//    kld();
//};

//#endif // KLD_H

#ifndef kld_sampling_hh
#define kld_sampling_hh

#include <vector>

using namespace std;


/**
   This class uses KL-Divergence to determine when a distribution has
   been adequately sampled.
**/
class kld_sampling {

public:
  kld_sampling();
  void init(float, float, const vector<float>&, int sample_min=absolute_min);
  int update(const vector<float>&);

private:
  static const int absolute_min=10;
  float confidence, max_error;
  vector<float> bin_size;

  static vector<float> ztable;
  int num_samples;
  vector< vector <float> > bins;

  vector<float> curr_sample;
  int support_samples, kld_samples;
  float zvalue;

  bool in_empty_bin();
  void build_table();
};

#endif
