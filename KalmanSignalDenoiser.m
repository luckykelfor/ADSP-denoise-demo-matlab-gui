function output=KalmanSignalDenoiser(Noisy,Clean,fs)

% OUTPUT=KALMANSIGNALDENOISER(NOISY,CLEAN,FS)
% this purpose of this function is to demonstrate the capability of kalman
% filter for denoising noisy speech (corrupted by white noise). Kalman
% filtering of noisy speech usually have two steps: 
% 1 . Estimating the AR parameters of speech segment
% 2 . Filtering the segment
% There are different approaches for extracting AR parameters of noisy
% speech in the literature, however, in this function none is implemented.
% Clean speech signal should be provided for this purpose.
%
% ARGUMENTS
% NOISY : noise contaminated speech
% CLEAN : clean speech signal
% FS    : Sampling frequency which should be the same for both signals
%
% Output is the denoised speech signal
%
% Required functions:
% SEGMENT
%Sep-04
%Esfandiar Zavarehei

W=fix(.025*fs); %Window length is 25 ms
SP=1; %Shift percentage is 40% (10ms) %Overlap-Add method works good with this value(.4)
SpecP=13;
Window=ones(W,1);

x=segment(Clean,W,SP,Window);
y=segment(Noisy,W,SP,Window);
n=segment(Noisy-Clean,W,SP,Window);
R=var(n);

H=[zeros(1,SpecP-1) 1];
G=H'; GGT=G*H;
FUpper=[zeros(SpecP-1,1) eye(SpecP-1)];
I=eye(SpecP);

[A Q]=lpc(x,SpecP);
P=diag(repmat(R(1),1,SpecP));

o=zeros(1,W*size(x,2));% allocating memory to the output in advance save a lot of computation time
o(1:SpecP)=y(1:SpecP,1)';
hwb = waitbar(0,'Please wait...','Name','¿¨¶ûÂüÂË²¨´¦Àí');
start=SpecP+1;
Sp=o(1:SpecP)';
t=SpecP+1;
for n=1:size(x,2)
    waitbar(n/size(x,2),hwb,['Please wait... ' num2str(fix(100*n/size(x,2))) ' %'])
    F=[FUpper; fliplr(-A(n,2:end))];% ×´Ì¬×ªÒÆ
    for i=start:W               
        S_=F*Sp;
        e=y(i,n)-S_(end);%innovation
        P_=F*P*F'+GGT*Q(n);    
        K=(P_*H')/(H*P_*H' + R(n));
        SOut=S_+K*e;        
        o(t-SpecP+1:t)=SOut'; %Notice that the previous SpecP-1 output samples are updated again
        P=(I-K*H)*P_;
        Sp=SOut;
        t=t+1;
    end
    start=1;
end
close(hwb)
output=o;






function FormantTrack=FTrackSig(signal,fs)

% F=FTRACKSIG(S)
% Formant Tracking 
% Inputs: S, signal, the speech signal with Motorola Format and sampling
% frequency fs (default=16000 KHz)
% Outputs: F, FormantTrack is a matrix each row of which is a track of one of
% the formants
%
% Important Variables in the program: There are several important variables
% in the program: 
%       Main Function (FTrackSig)
%           MaxFormantFreq : The maximum frequency of the formant
%           MaxBWFreq : The maximum BandWidth of the formant
%           SegLength: Number of samplein each frame, 
%           SP: Shift Percentage of frames, (1-SP is overlap percentage)
%           NumberOfFormants: The number of formants to be chosen
%           PreEmphFact: the pre-emphasis factor
%           fs: sampling frequency
%       FormantCand function
%           P: order of LPC model
%---------------------------------- 
% What does this program do:
% The speech signal is first choped into segments and the pre-emphasize,
% then windowed. Then the LPC Model for these segments are extracted. using
% these coefficients the roor=ts of the LP model is calculated, sorted by
% increasing frequency and the extreme poles are eliminated. (an extreme
% ploe is one with zero frequency or out of range frequency, or  rather
% large bandwidth). if the number of poles are not sufficient the segments
% will be re modeled using a larger order for LPC (THIS IS STILL TO BE
% DISCUSSED ACTUALLY). these sorted poles are then grouped with regards to
% the number of poles required. if there were more than one group each
% group will be evaluated with regards to the different distributions of
% phonems which are loaded from another file, and then evaluated in terms
% of the trajectory continueity. the best formant will be chosen then using
% these evaluations.
%
%
% Author Esfandiar Zavarehei
% Date 5-Nov-2003
% Revised: 19-Nov-2003
% Revised: 21-Nov-2003
%
%
%       Notes:
%       The MAXIMUM FREQUENCY is a crucial parameter if you have a
%       low sampling frequency you have to adjust it your self.
%       The NUMBER OF FORMANTS is 5 if you have a low sampling frequency
%       try in decrease this number.

if nargin<2
    fs=16000; % The Sampling frequency
end

SegLength=.025*fs; % This is equal to 25ms
SP=.4; % Shift Percentage
PreEmphFact=-0.98;
NumberOfFormants=5;
MaxFormantFreq=5000;
MaxBWFreq=600;
FreqRes=512;

sigLength=length(signal); 

NumSeg=fix(sigLength-SegLength+SegLength*SP)/(SegLength*SP);
signal=real(filter([1 PreEmphFact],1,signal));    %Pre-emphasizing

BegPtr=1;
FCFreq=[];
FCBW=[];
hamwin=hamming(SegLength);
AddP=1;

for n=1:NumSeg   
    segment=signal(BegPtr:(BegPtr+SegLength-1));        %Chopping    
    segment=segment.*hamwin;                            %Windowing
    segment=[segment;zeros((FreqRes-SegLength),1)];     %adjusting frequency resoloution
    Candidates=FormantCand(segment);
    FCseg=angle(Candidates)*fs/(2*pi);
    FCBW=abs(log(abs(Candidates))*fs/pi);
    FCseg=FCseg(find(FCBW<MaxBWFreq));
    FCseg=FCseg(find(FCseg<MaxFormantFreq));    
    while (NumberOfFormants>length(FCseg))
        Candidates=FormantCand(segment,AddP);
        FCseg=angle(Candidates)*fs/(2*pi);
        FCBW=abs(log(abs(Candidates))*fs/pi);
        FCseg=FCseg(find(FCBW<MaxBWFreq));
        FCseg=FCseg(find(FCseg<MaxFormantFreq));
        AddP=AddP+1;
    end
    
    if (n>1)
        Formants=distFormant(FCseg,NumberOfFormants,LastFormant);
    else
        Formants=distFormant(FCseg,NumberOfFormants);
    end
    ForMat=Formants.FM;
    chance=1./Formants.dst;

    [a b]=max(chance);
    LastFormant=ForMat(:,b);
    FormantTrack(:,n)=LastFormant;    
    
    AddP=1;
    BegPtr=BegPtr+fix(SegLength*SP);              %Moving the pointer to next segment

end

%-----------------------------extracting, and choosing the appropriate poles of each segment
function Candidates=FormantCand(segment,AddP)
%5 NOV 2003
%This Program returns the Formant candidate of a segment
%the segment, which is probably a piece of speech signal,
%is not windowed, pre-emphasized, zero padded, etc. in this program.
if (nargin<2)
    AddP=0; %The number to be added to the LP Order, if it did not have enough Poles extracted
end
P=15+AddP; %number of LPC Coefitions
lpccof=lpc(segment,P);

rts=roots(lpccof);

Phase=angle(rts);
Amp=abs(rts);
posPhaseIndex=find((Phase>0.000001) & (Phase<(pi-0.000001)));
Phase=Phase(posPhaseIndex);
Amp=Amp(posPhaseIndex);
[Phase sIndex]=sort(Phase);
Amp=Amp(sIndex);

%Candidates
Candidates=Amp.*exp(j*Phase);

%-------------------------------------------DISTFORMANT
function dist=distFormant(Poles,numberOfFormants,LastChosenFormant)
%THIS FUNCTION CALCULATE THE DISTANCE THE LAST FORMANT WITH ALL AVAILABLE
%CANDIDATES AND RETURN A VECTOR (ROW VWCTOR) THAT CONTAINS THESE DISTANCES.
%Formants are row vectors in this program

% if (length(Poles)<numberOfFormants)
%     dist.FM=[Poles;NaN*ones(numberOfFormants-length(Poles),1)];
%     dist.dst=1;
%     return
% end

FormantMat=(nchoosek(Poles,numberOfFormants))';
numberOfCands=size(FormantMat,2);
if (nargin==3)
    dst=sum((FormantMat-LastChosenFormant(:,ones(1,numberOfCands))).^2);
elseif (nargin==2)
    dst=ones(1,size(FormantMat,2));
else
    error('not appropriate number of inputs')
end
dist.FM=FormantMat;
dist.dst=dst;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
function vec=mat2vec(mat)
% VEC=MAT2VEC(MAT) 
% This function augments the rows of the input matrix MAT 
% forming a vector VEC
%
% Example:
%
% A= [1 2 1 6
%     2 4 1 9];
% B = mat2vec(MAT);
% Then Be will be:
% B=[1 2 1 6 2 4 1 9];
%
%Sep-04
%Esfandiar Zavarehei

numofrow=size(mat,1);
vec=[];
for i=1:numofrow
    vec=[vec mat(i,:)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ReconstructedSignal=OverlapAdd(XNEW,yphase,windowLen,ShiftLen);

%Y=OverlapAdd(X,A,W,S);
%Y is the signal reconstructed signal from its spectrogram. X is a matrix
%with each column being the fft of a segment of signal. A is the phase
%angle of the spectrum which should have the same dimension as X. if it is
%not given the phase angle of X is used which in the case of real values is
%zero (assuming that its the magnitude). W is the window length of time
%domain segments if not given the length is assumed to be twice as long as
%fft window length. S is the shift length of the segmentation process ( for
%example in the case of non overlapping signals it is equal to W and in the
%case of %50 overlap is equal to W/2. if not givven W/2 is used. Y is the
%reconstructed time domain signal.
%Sep-04
%Esfandiar Zavarehei

if nargin<2
    yphase=angle(XNEW);
end
if nargin<3
    windowLen=size(XNEW,1)*2;
end
if nargin<4
    ShiftLen=windowLen/2;
end


[FreqRes FrameNum]=size(XNEW);

sig=zeros((FrameNum-1)*ShiftLen+windowLen,1);
for i=1:FrameNum
    start=(i-1)*ShiftLen+1;
    spec=[XNEW(:,i).*exp(j*yphase(:,i))];
    spec=[spec;flipud(conj(spec))];    
    sig(start:start+windowLen-1)=sig(start:start+windowLen-1)+real(ifft(spec,windowLen));   
end
ReconstructedSignal=sig;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
function Seg=segment(signal,W,SP,Window)

% SEGMENT chops a signal to overlapping windowed segments
% A= SEGMENT(X,W,SP,WIN) returns a matrix which its columns are segmented
% and windowed frames of the input one dimentional signal, X. W is the
% number of samples per window, default value W=256. SP is the shift
% percentage, default value SP=0.4. WIN is the window that is multiplied by
% each segment and its length should be W. the default window is hamming
% window.
% 06-Sep-04
% Esfandiar Zavarehei

if nargin<3
    SP=.4;
end
if nargin<2
    W=256;
end
if nargin<4
    Window=hamming(W);
end

L=length(signal);
SP=fix(W.*SP);
N=fix((L-W)/SP +1); %number of segments

Index=(repmat(1:W,N,1)+repmat((0:(N-1))'*SP,1,W))';
hw=repmat(Window,1,N);
Seg=signal(Index).*hw;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [NoiseFlag, SpeechFlag, NoiseCounter, Dist]=vad(signal,noise,NoiseCounter,NoiseMargin,Hangover)

%[NOISEFLAG, SPEECHFLAG, NOISECOUNTER, DIST]=vad(SIGNAL,NOISE,NOISECOUNTER,NOISEMARGIN,HANGOVER)
%Spectral Distance Voice Activity Detector
%SIGNAL is the the current frames magnitude spectrum which is to be labeld as
%noise or speech, NOISE is noise magnitude spectrum template (estimation),
%NOISECOUNTER is the number of imediate previous noise frames, NOISEMARGIN
%(default 3)is the spectral distance threshold. HANGOVER ( default 8 )is
%the number of noise segments after which the SPEECHFLAG is reset (goes to
%zero). NOISEFLAG is set to one if the the segment is labeld as noise
%NOISECOUNTER returns the number of previous noise segments, this value is
%reset (to zero) whenever a speech segment is detected. DIST is the
%spectral distance.
%Note that:
%           1. This function works on a frame-by-frame basis not the whole
%           signal. If you would like to get the flags for the entire wave
%           signal you can use SEGMENT function and apply VAD to each
%           segment
%           2. It is safe to use the segment for updating noise when
%           SPEECHFLAG is zero. NOISEFLAG would have been 1 for 8 frames
%           before that happens.
%Author: Saeed Vaseghi
%edited by Esfandiar Zavarehei
%Sep-04

if nargin<4
    NoiseMargin=3;
end
if nargin<5
    Hangover=8;
end
if nargin<3
    NoiseCounter=0;
end
    
FreqResol=length(signal);

SpectralDist= 20*(log10(signal)-log10(noise));
SpectralDist(find(SpectralDist<0))=0;

Dist=mean(SpectralDist); 
if (Dist < NoiseMargin) 
    NoiseFlag=1; 
    NoiseCounter=NoiseCounter+1;
else
    NoiseFlag=0;
    NoiseCounter=0;
end

% Detect noise only periods and attenuate the signal     
if (NoiseCounter > Hangover) 
    SpeechFlag=0;    
else 
    SpeechFlag=1; 
end 