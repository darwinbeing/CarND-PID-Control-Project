#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <string>

class Twiddle {
  enum class State {S1, S2, S3};

 public:
  // Vector of parameters in scale of change
  std::vector<double> p_;
  std::vector<double> dp_;

  double error_;

  int p_index_;

  // Best error
  double best_error_;

  // Tolerance used to stop
  double tolerance_;

  State state_;

  bool tune_;

  std::string type_;

  const int kMaxSteps = 8;
  /*
   * Constructor
   */
  Twiddle();

  /*
   * Destructor.
   */
  virtual ~Twiddle();

  /*
   * Initialize the optimizer
   */
  void Init(double tol, const std::vector<double>& p, const std::vector<double>& dp, const std::string type);

  /*
   * Returns a set of parameters to test given the error in the last trial
   */
  std::vector<double> getParams();

  void UpdateError(double error);

  /*
   * Returns true if the parameters have already been tuned
   */
  bool isDone();

 private:
  // check whether the twiddle was initialized or not (first measurement)
  bool is_initialized_;
  int steps_;
};

#endif // TWIDDLE_H
