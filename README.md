Code is in `main.m`.

All other are helper functions.

# GUIDE to reproduce problems. 
1. Start from mode = `pos_control` to verify this model works
2. Go to mode = `TO_control`. -> Inequality is at 2.5e-1
3. Go set same upper-lower constr in section 2: ubx_0,lbx_0 ->  Converges
4. Comment out line 87 `ocp_model.set('constr_x0', [x0;Tmid])`  and set  diffent ubx_0,lbx_0 -> QP failure