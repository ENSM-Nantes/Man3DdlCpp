arg_list = argv();
maneuver = arg_list{1};

traj_solv_csv = "./plot/savePlotTraj.csv";

if maneuver == '1'
  traj_emp_csv = "./emp_data/traj_ZZ10.dat";
elseif maneuver == '2'
  traj_emp_csv = "./emp_data/traj_TC35.dat";   
endif

ref_data = csvread(traj_solv_csv, 0, 0);
ref_data_emp = csvread(traj_emp_csv, 0, 0);

figure();
subplot (2, 2, 1)
hold on;
plot(ref_data(:, 1), ref_data(:, 2), "r.", "markersize", 10);
plot(ref_data_emp(:, 1), ref_data_emp(:, 2), "b.", "markersize", 10);
title("Trajectory");
xlabel("x / lpp");
ylabel("y / lpp");

U_solv_csv = "./plot/savePlotU.csv";

if maneuver == '1'
  U_emp_csv = "./emp_data/U_ZZ10.dat";
elseif maneuver == '2'
  U_emp_csv = "./emp_data/U_TC35.dat";   
endif

ref_data = csvread(U_solv_csv, 0, 0);
ref_data_emp = csvread(U_emp_csv, 0, 0);

#figure("name","U");
subplot (2, 2, 2)
hold on;
plot(ref_data(:, 1), ref_data(:, 2), "r.", "markersize", 10);
plot(ref_data_emp(:, 1), ref_data_emp(:, 2), "b.", "markersize", 10);
title("U");
xlabel("t * u_0 / L_pp");
ylabel("U / u_0");

R_solv_csv = "./plot/savePlotR.csv";

if maneuver == '1'
  R_emp_csv = "./emp_data/r_ZZ10.dat";
elseif maneuver == '2'
  R_emp_csv = "./emp_data/r_TC35.dat";   
endif

ref_data = csvread(R_solv_csv, 0, 0);
ref_data_emp = csvread(R_emp_csv, 0, 0);

#figure("name","R");
subplot (2, 2, 3)
hold on;
plot(ref_data(:, 1), ref_data(:, 2), "r.", "markersize", 10);
plot(ref_data_emp(:, 1), ref_data_emp(:, 2), "b.", "markersize", 10);
title("R");
xlabel("t * u_0 / L_pp");
ylabel("r * L_pp / u_0");

Drift_solv_csv = "./plot/savePlotDrift.csv";

if maneuver == '1'
  Drift_emp_csv = "./emp_data/beta_ZZ10.dat";
elseif maneuver == '2'
  Drift_emp_csv = "./emp_data/beta_TC35.dat";   
endif

ref_data = csvread(Drift_solv_csv, 0, 0);
ref_data_emp = csvread(Drift_emp_csv, 0, 0);


#figure("name","Drift Angle");
subplot (2, 2, 4)
hold on;
plot(ref_data(:, 1), ref_data(:, 2), "r.", "markersize", 10);
plot(ref_data_emp(:, 1), ref_data_emp(:, 2), "b.", "markersize", 10);
title("Drift Angle");
xlabel("t * u_0 / L_pp");
ylabel("beta deg");

if maneuver == '1'

  Rudd_solv_csv = "./plot/savePlotRudd.csv";
  Rudd_emp_csv = "./emp_data/delta_ZZ10.dat";
  ref_data = csvread(Rudd_solv_csv, 0, 0);
  ref_data_emp = csvread(Rudd_emp_csv, 0, 0);

  figure();
  subplot (2, 2, 1)
  hold on;
  plot(ref_data(:, 1), ref_data(:, 2), "r.", "markersize", 10);
  plot(ref_data_emp(:, 1), ref_data_emp(:, 2), "b.", "markersize", 10);
  title("Rudder Angle");
  xlabel("t * u_0 / L_pp");
  ylabel("delta deg");

  Head_solv_csv = "./plot/savePlotHeading.csv";
  Head_emp_csv = "./emp_data/psi_ZZ10.dat";
  ref_data = csvread(Head_solv_csv, 0, 0);
  ref_data_emp = csvread(Head_emp_csv, 0, 0);

  subplot (2, 2, 2)
  hold on;
  plot(ref_data(:, 1), ref_data(:, 2), "r.", "markersize", 10);
  plot(ref_data_emp(:, 1), ref_data_emp(:, 2), "b.", "markersize", 10);
  title("Heading");
  xlabel("t * u_0 / L_pp");
  ylabel("psi deg");

endif


ginput()
