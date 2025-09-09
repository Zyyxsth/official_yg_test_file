
namespace px4_ctrl{
  class PIDctrl{
    
    public:
      PIDctrl();
      PIDctrl(double dt);
      ~PIDctrl();

      void setParams(double kp, double ki, double kd, double v_max, double a_max);
      
      void setVPrev(double v_prev);
      void setPErrPrev(double p_err_prev);
      
      double getPErrPrev();
      double getVPrev();

      double velCtrl(double p_err);

    private:
      double kp_, ki_, kd_, dt_;
      double v_max_, a_max_;
      double p_err_prev_, v_prev_;
      double p_int_, p_dif_;
      double v_ref_;

  };

}