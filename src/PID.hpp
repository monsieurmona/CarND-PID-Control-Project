/**
  * Author: Mario Lüder
  */
#ifndef PID_HPP
#define PID_HPP

// system includes
#include <algorithm>
#include <stddef.h>

class PID {
public:
   /*
  * Constructor
  */
   PID();

   /*
  * Destructor.
  */
   virtual ~PID();

   /*
  * Initialize PID.
  */
   void init(const double p, const double i, const double d,
             const double step_p, const double step_i, const double step_d);

   /**
   * @brief reset PID errors
   */
   void resetError();

   /**
   * @brief execute pid controller
   * @param cte cross track error (such as distance between a given track and position)
   * @return compensation value (such as steering angle)
   */
   double run(const double cte, bool & twiddled);

   bool doTwiddle = true;

private:
   /**
   * @brief finds the coefficients (tau) for PID
   * @param cte cross track error (such as distance between a given track and position)
   */
   void twiddle();

   /**
   * @brief sum of array
   * @param arr
   * @param length array length
   * @return sum of array values
   */
   double sum(const double arr[], const size_t length) const;

   static constexpr size_t m_nTau = 3;

   /*
  * Errors
  */
   struct TauError
   {
      void reset()
      {
         m_pError = 0.0;
         m_iError = 0.0;
         m_dError = 0.0;
      }

      void add(const double cte)
      {
         m_dError = cte - m_pError;
         m_pError = cte;
         m_iError += cte;
      }

      void setMax()
      {
         m_dError = std::numeric_limits<double>::max();
         m_pError = std::numeric_limits<double>::max();
         m_iError = std::numeric_limits<double>::max();
      }

      double m_pError = 0.0;
      double m_dError = 0.0;
      double m_iError = 0.0;
   };

   union UTauError
   {
      UTauError() { values.reset(); }
      TauError values;
      double valuesArray[m_nTau];
   };

   UTauError m_tauError;

   static constexpr double const init_p = 1.0;
   static constexpr double const init_i = 0.01;
   static constexpr double const init_d = 0.5;

   /*
  * Coefficients (tau)
  */
   struct Tau
   {
      void reset()
      {
         m_p = init_p;
         m_i = init_i;
         m_d = init_d;
      }

      void set(const double p, const double i, const double d)
      {
         m_p = p;
         m_i = i;
         m_d = d;
      }

      double m_p = init_p;
      double m_d = init_d;
      double m_i = init_i;
   };

   union UTau
   {
      UTau() { values.reset(); }

      Tau values;
      double valuesArray[m_nTau];
   };

   UTau m_tau;

   /*
   * Twiddle parameters
   */
   double m_steps[m_nTau] = { 0.1, 0.1, 0.001};

   const double m_twiddleTolerance[m_nTau] =
      {0.0005, 0.01, 0.00001}; // p, d, i
   size_t m_roundIdx = 0;
   const size_t m_twiddleRound = 1400;
   const size_t m_twiddleFreeRun = 0;
   double m_twiddleRoundError = 0.0;
   double m_bestError = std::numeric_limits<double>::max();
   bool m_resetTau[m_nTau] = {false, false, false };
   size_t m_tauIdx = 0;
   bool m_finished = false;
   size_t freeRunIdx = 0;
};

#endif /* PID_HPP */

