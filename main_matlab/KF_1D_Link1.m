function [KalmanAngle,KalmanUncertainty_output] = KF_1D_Link1(KalmanState,KalmanUncertainty,KalmanInput,KalmanMeasurement,Ts,q,r)

% Variáveis que devem ser pré setadas antes de chamar essa função
%{
KalmanAngleRoll = 0;
KalmanAnglePitch = 0;
KalmanUncertaintyAngleRoll = 2*2;
KalmanUncertaintyAnglePitch = 2*2;
Kalman1DOutput = [0 0];
%}

KalmanState = KalmanState + Ts*KalmanInput;
KalmanUncertainty = KalmanUncertainty + (Ts^2)*(q^2);
KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + r^2);
KalmanState = KalmanState + KalmanGain * (KalmanMeasurement-KalmanState);
KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;

KalmanAngle = KalmanState; 
KalmanUncertainty_output = KalmanUncertainty;

end