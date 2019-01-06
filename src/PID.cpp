/**
  * Author: Mario Lüder
  */

// system includes
#include <iostream>

// local includes
#include "PID.hpp"

PID::PID()
{}

PID::~PID() {}

void PID::init(const double p, const double i, const double d,
               const double step_p, const double step_i, const double step_d)
{
   m_tauError.values.reset();
   m_tau.values.set(p, i, d);

   m_steps[0] = step_p;
   m_steps[1] = step_d;
   m_steps[2] = step_i;

   m_roundIdx = 0;
   m_twiddleRoundError = 0.0;
   m_bestError = std::numeric_limits<double>::max();

   for (size_t i = 0; i < m_nTau; ++i)
   {
      m_resetTau[i] = false;
   }

   m_tauIdx = 0;

   m_finished = false;
}

void PID::resetError()
{
   m_tauError.values.reset();
}

double PID::run(const double cte, bool & twiddled)
{
   twiddled = false;
   m_tauError.values.add(cte);

   // the time intervall should be considered here, but was not given, so dt is assumed to 1.0
   const double p = m_tau.values.m_p;
   const double i = m_tau.values.m_i;
   const double d = m_tau.values.m_d;

   const double p_err = m_tauError.values.m_pError;
   const double i_err = m_tauError.values.m_iError;
   const double d_err = m_tauError.values.m_dError;

   const double result = -p * p_err - d * d_err - i * i_err;

   // twiddle
   if (doTwiddle)
   {
      if (m_roundIdx == 0)
      {
         twiddled = true;
         m_roundIdx++;
         return result;
      }

      if (freeRunIdx >= m_twiddleFreeRun)
      {
         m_twiddleRoundError += pow(cte,2);
      }

      freeRunIdx++;

      if ((m_roundIdx > 0) && ((m_roundIdx % m_twiddleRound) == 0))
      {
         if (m_roundIdx == m_twiddleRound)
         {
            m_bestError = m_twiddleRoundError;

            // increase the p for twiddle
            // this is needed only at the first time
            // all other values will be increase by twiddle
            m_tau.values.m_p += m_steps[0];
         }
         else
         {
            twiddle();
         }

         m_twiddleRoundError = 0.0;
         m_tauError.values.m_iError = 0.0;
         twiddled = true;
         freeRunIdx = 0;
      }

      m_roundIdx++;
   }

   return result;
}

void PID::twiddle()
{
   if (m_finished)
   {
      return;
   }

   size_t & tauIdx = m_tauIdx;
   const size_t thisTauIdx = tauIdx;
   //for (size_t tauIdx = 0; tauIdx < m_nTau; ++tauIdx)
   //{
      double & tau           = m_tau.valuesArray[tauIdx];
      double & step          = m_steps[tauIdx];
      const double & error   = m_twiddleRoundError;
      double & bestError     = m_bestError;
      bool   & resetTau      = m_resetTau[tauIdx];
      const double minStep   = m_twiddleTolerance[tauIdx];
      const bool isMinStep   = step <= minStep;
      const bool isBestError = error < bestError;

      if (isMinStep || isBestError)
      {
         // already best parameter found
         if (isMinStep)
         {
            std::cout << "Best parameter found for tau:"
                      << tauIdx << "(p=0, d=1, i=2)" << std::endl;
         }
         else
         {
            std::cout << "Better tau:" << tauIdx << "(p=0, d=1, i=2)" << std::endl;
            bestError = error;
         }

         m_finished = true;

         // tune the next parameter only after we have a better parameter
         for (size_t parameterCount = 0; parameterCount < m_nTau; ++parameterCount)
         {
            tauIdx = (tauIdx +1 ) % m_nTau;
            resetTau = false;

            double & nextTau     = m_tau.valuesArray[tauIdx];
            double & nextStep    = m_steps[tauIdx];
            const double nextMinStep = m_twiddleTolerance[tauIdx];

            if (nextStep > nextMinStep)
            {
               nextStep *= 1.1;
               nextTau += nextStep;
               m_finished = false;
               break;
            }
         }

         if (m_finished)
         {
            std::cout << "Twiddle Finished" << std::endl;
            return;
         }
      }
      else
      {
         if (resetTau)
         {
            tau += step; // reset
            step *= 0.9;
            resetTau = false;
            tau += step; // for the next step
         }
         else
         {
            tau -= 2 * step;

            // if the error is in next round not smaller
            // reset tau
            resetTau = true;
         }
      }
   //}

   if (sum(m_steps, m_nTau) < sum(m_twiddleTolerance,m_nTau))
   {
      std::cout << "BEST ";
   }

   std::cout << "Twiddle. tau:" << thisTauIdx << " (p=0, d=1, i=2)" << std::endl
             << "   tau constants." << std::endl
             << "      p:" << m_tau.values.m_p
             << " i:" << m_tau.values.m_i
             << " d:" << m_tau.values.m_d << std::endl
             << "   steps." << std::endl
             << "      p step:" << m_steps[0]
             << " i step:" << m_steps[2]
             << " d step:" << m_steps[1] << std::endl
             << "   error:" << m_twiddleRoundError << std::endl
             << "   bestError:" << m_bestError << std::endl
             << "   reset:" << std::endl
             << "      p reset:" << m_resetTau[0]
             << " i reset:" << m_resetTau[2]
             << " d reset:" << m_resetTau[1] << std::endl;
}

double PID::sum(const double arr[], const size_t length) const
{
   double result = 0.0;

   for (size_t i = 0; i < length; ++i)
   {
      result += arr[i];
   }

   return result;
}
