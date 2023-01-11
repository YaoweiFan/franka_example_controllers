#include <franka_example_controllers/p2p.h>
#include<cmath>

void p2p(double time, double sample_time, double position_end,
double max_velocity,double max_accelerate, double y[4]){
    // double            y[4];
    double            v_M = max_velocity;
	double            a_M = max_accelerate;
	double            p_end = position_end;
    double			  ts = sample_time;
	double			  t = time;
	double      q0,T,tt,rt,rrt;
	double		j_M, delay, freq, pause;

    j_M	  = a_M*20.0;   /* Max jerk (the rate of acceleration) is set as 20*Max acceleration */
	delay = 4;       	/*2 output trajectory is set as zero in the peroid of delay */
	freq  = 1.0/ts;     /* sampling frequency =  */
	pause = 4;			/*2 when the trajectory moves to zero position and final position, pause 0.4*2 sec */
	
	double		t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17;
	double		a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15;
	double		v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15;
	double		s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16;
	double		T0,T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13,T14,T15,T16,T17;
	
	
	
	/* compute initial condition of each trajectory segment */
    t0  = pause;
    
	t1  = a_M/j_M;
	t2  = v_M/a_M - a_M/j_M;
	t3  = a_M/j_M;

	a1 = j_M*t1;
	a2 = a1;
	a3 = a2 - j_M*t3;

	v1 = j_M*t1*t1/2.0;
	v2 = v1 + a1*t2;
	v3 = v2 + a2*t3 - j_M*t3*t3/2.0;

    s1 = j_M*t1*t1*t1/6;
    s2 = s1 + v1*t2 + a1*t2*t2/2.0;
	s3 = s2 + v2*t3 + a2*t3*t3/2.0 - j_M*t3*t3*t3/6.0;

	t4  = (p_end - 2.0*s3)/v_M;
	t5  = a_M/j_M;
	t6  = v_M/a_M - a_M/j_M;
	t7  = a_M/j_M;
	t8  = pause;
    t9  = pause;
	t10 = a_M/j_M;
	t11 = v_M/a_M - a_M/j_M;
	t12 = a_M/j_M;
	t13 = (p_end - 2.0*s3)/v_M;
	t14 = a_M/j_M;
	t15 = v_M/a_M - a_M/j_M;
	t16 = a_M/j_M;
	t17 = pause;

	a4  = a3;
	a5  = a4 - j_M*t5;
	a6  = a5;
	a7  = a6 + j_M*t7;
	a8  = a7;
	a9  = a8;
	a10 = a9 - j_M*t10;
	a11 = a10;
	a12 = a11 + j_M*t12;
	a13 = a12;
	a14 = a13 + j_M*t14;
	a15 = a14;

	v4  = v3 + a3*t4;
	v5  = v4 + a4*t5 - j_M*t5*t5/2.0;
	v6  = v5 + a5*t6;
	v7  = v6 + a6*t7 + j_M*t7*t7/2.0;
	v8  = v7 + a7*t8;
	v9  = v8 + a8*t9;
	v10 = v9 + a9*t10 - j_M*t10*t10/2.0;
	v11 = v10 + a10*t11;
	v12 = v11 + a11*t12 + j_M*t12*t12/2.0;
	v13 = v12 + a12*t13;
	v14 = v13 + a13*t14 + j_M*t14*t14/2.0;
	v15 = v14 + a14*t15;

	s4  = s3 + v3*t4 + a3*t4*t4/2.0;
	s5  = s4 + v4*t5 + a4*t5*t5/2.0 - j_M*t5*t5*t5/6.0;
	s6  = s5 + v5*t6 + a5*t6*t6/2.0;
	s7  = s6 + v6*t7 + a6*t7*t7/2.0 + j_M*t7*t7*t7/6.0;
	s8  = s7 + v7*t8 + a7*t8*t8/2.0;
	s9  = s8 + v8*t9 + a8*t9*t9/2.0;
	s10 = s9 + v9*t10 + a9*t10*t10/2.0 - j_M*t10*t10*t10/6.0;
	s11 = s10 + v10*t11 + a10*t11*t11/2.0;
	s12 = s11 + v11*t12 + a11*t12*t12/2.0 + j_M*t12*t12*t12/6.0;
	s13 = s12 + v12*t13 + a12*t13*t13/2.0;
	s14 = s13 + v13*t14 + a13*t14*t14/2.0 + j_M*t14*t14*t14/6.0;
	s15 = s14 + v14*t15 + a14*t15*t15/2.0;
	s16 = s15 + v15*t16 + a15*t16*t16/2.0 - j_M*t16*t16*t16/6.0;

	T0  = t0;
	T1  = T0 + t1;
	T2  = T1 + t2;
	T3  = T2 + t3;
	T4  = T3 + t4;
	T5  = T4 + t5;
	T6  = T5 + t6;
	T7  = T6 + t7;
	T8  = T7 + t8;
	T9  = T8 + t9;
	T10 = T9 + t10;
	T11 = T10 + t11;
	T12 = T11 + t12;
	T13 = T12 + t13;
	T14 = T13 + t14;
	T15 = T14 + t15;
	T16 = T15 + t16;
	T17 = T16 + t17;

  /* generate trajectory */
  if(t<=delay)
		{y[3] = 0.0;		/* jerk */
		 y[2] = 0.0;		/* acceleration */
		 y[1] = 0.0;		/* velocity */
	     y[0] = 0.0;		/* position */
		}

  else
	{
	 q0	  = 0;//*uPtrs[0];
	 T	  = T17*freq;
     tt   = (t-delay)*freq;
     rt   = fmod(tt,T);
     rrt  = rt/freq;


    if(rt<=T0*freq)
		{y[3] = 0.0;
		 y[2] = 0.0;
		 y[1] = 0.0;
		 y[0] = 0.0;
		}

    if(rt>T0*freq & rt<=T1*freq)
		{y[3] = j_M;
		 y[2] = j_M*(rrt-T0);
		 y[1] = j_M*(rrt-T0)*(rrt-T0)/2.0;
		 y[0] = j_M*(rrt-T0)*(rrt-T0)*(rrt-T0)/6.0;
		}

	if(rt>T1*freq & rt<=T2*freq)
		{y[3] = 0.0;
		 y[2] = a1;
		 y[1] = v1 + a1*(rrt-T1);
		 y[0] = s1 + v1*(rrt-T1) + a1*(rrt-T1)*(rrt-T1)/2.0;
		}

	if(rt>T2*freq & rt<=T3*freq)
		{y[3] = -j_M;
		 y[2] = a2 - j_M*(rrt-T2);
		 y[1] = v2 + a2*(rrt-T2) - j_M*(rrt-T2)*(rrt-T2)/2.0;
		 y[0] = s2 + v2*(rrt-T2) + a2*(rrt-T2)*(rrt-T2)/2.0 - j_M*(rrt-T2)*(rrt-T2)*(rrt-T2)/6.0;
		}

	if(rt>T3*freq & rt<=T4*freq)
		{y[3] = 0.0;
		 y[2] = a3;
		 y[1] = v3 + a3*(rrt-T3);
		 y[0] = s3 + v3*(rrt-T3) + a3*(rrt-T3)*(rrt-T3)/2.0;
		}

	if(rt>T4*freq & rt<=T5*freq)
		{y[3] = -j_M;
		 y[2] = a4 - j_M*(rrt-T4);
		 y[1] = v4 + a4*(rrt-T4) - j_M*(rrt-T4)*(rrt-T4)/2.0;
		 y[0] = s4 + v4*(rrt-T4) + a4*(rrt-T4)*(rrt-T4)/2.0 - j_M*(rrt-T4)*(rrt-T4)*(rrt-T4)/6.0;
		}

    if(rt>T5*freq & rt<=T6*freq)
		{y[3] = 0;
		 y[2] = a5;
		 y[1] = v5 + a5*(rrt-T5);
		 y[0] = s5 + v5*(rrt-T5) + a5*(rrt-T5)*(rrt-T5)/2.0;
		}

    if(rt>T6*freq & rt<=T7*freq)
		{y[3] = j_M;
		 y[2] = a6 + j_M*(rrt-T6);
		 y[1] = v6 + a6*(rrt-T6) + j_M*(rrt-T6)*(rrt-T6)/2.0;
		 y[0] = s6 + v6*(rrt-T6) + a6*(rrt-T6)*(rrt-T6)/2.0 + j_M*(rrt-T6)*(rrt-T6)*(rrt-T6)/6.0;
		}

	if(rt>T7*freq & rt<=T8*freq)
		{y[3] = 0.0;
		 y[2] = 0.0;
		 y[1] = 0.0;
		 y[0] = s7;
		}

	if(rt>T8*freq & rt<=T9*freq)
		{y[3] = 0.0;
		 y[2] = 0.0;
		 y[1] = 0.0;
		 y[0] = s8;
		}

	if(rt>T9*freq & rt<=T10*freq)
		{y[3] = -j_M;
		 y[2] = a9 - j_M*(rrt-T9);
		 y[1] = v9 + a9*(rrt-T9) - j_M*(rrt-T9)*(rrt-T9)/2.0;
		 y[0] = s9 + v9*(rrt-T9) + a9*(rrt-T9)*(rrt-T9)/2.0 - j_M*(rrt-T9)*(rrt-T9)*(rrt-T9)/6.0;
		}

	if(rt>T10*freq & rt<=T11*freq)
		{y[3] = 0.0;
		 y[2] = a10;
		 y[1] = v10 + a10*(rrt-T10);
		 y[0] = s10 + v10*(rrt-T10) + a10*(rrt-T10)*(rrt-T10)/2.0;
		}


   if(rt>T11*freq & rt<=T12*freq)
		{y[3] = j_M;
		 y[2] = a11 + j_M*(rrt-T11);
		 y[1] = v11 + a11*(rrt-T11) + j_M*(rrt-T11)*(rrt-T11)/2.0;
		 y[0] = s11 + v11*(rrt-T11) + a11*(rrt-T11)*(rrt-T11)/2.0 + j_M*(rrt-T11)*(rrt-T11)*(rrt-T11)/6.0;
		}
  
   if(rt>T12*freq & rt<=T13*freq)
		{y[3] = 0;
		 y[2] = a12;
		 y[1] = v12 + a12*(rrt-T12);
		 y[0] = s12 + v12*(rrt-T12) + a12*(rrt-T12)*(rrt-T12)/2.0;
		}

   if(rt>T13*freq & rt<=T14*freq)
		{y[3] = j_M;
		 y[2] = a13 + j_M*(rrt-T13);
		 y[1] = v13 + a13*(rrt-T13) + j_M*(rrt-T13)*(rrt-T13)/2.0;
		 y[0] = s13 + v13*(rrt-T13) + a13*(rrt-T13)*(rrt-T13)/2.0 + j_M*(rrt-T13)*(rrt-T13)*(rrt-T13)/6.0;
		}

   if(rt>T14*freq & rt<=T15*freq)
		{y[3] = 0.0;
		 y[2] = a14;
		 y[1] = v14 + a14*(rrt-T14);
		 y[0] = s14 + v14*(rrt-T14) + a14*(rrt-T14)*(rrt-T14)/2.0;
		}

   if(rt>T15*freq & rt<=T16*freq)
		{y[3] = -j_M;
		 y[2] = a15 - j_M*(rrt-T15);
		 y[1] = v15 + a15*(rrt-T15) - j_M*(rrt-T15)*(rrt-T15)/2.0;
		 y[0] = s15 + v15*(rrt-T15) + a15*(rrt-T15)*(rrt-T15)/2.0 - j_M*(rrt-T15)*(rrt-T15)*(rrt-T15)/6.0;
		}

   if(rt>T16*freq & rt<=T17*freq)
		{y[3] = 0.0;
		 y[2] = 0.0;
		 y[1] = 0.0;
		 y[0] = s16;
		}
    }
    
}