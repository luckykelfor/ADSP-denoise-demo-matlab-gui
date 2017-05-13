function varargout = MyNoiseSuppressionDemo(varargin)
% MYNOISESUPPRESSIONDEMO MATLAB code for MyNoiseSuppressionDemo.fig
%      MYNOISESUPPRESSIONDEMO, by itself, creates a new MYNOISESUPPRESSIONDEMO or raises the existing
%      singleton*.
%
%      H = MYNOISESUPPRESSIONDEMO returns the handle to a new MYNOISESUPPRESSIONDEMO or the handle to
%      the existing singleton*.
%
%      MYNOISESUPPRESSIONDEMO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYNOISESUPPRESSIONDEMO.M with the given input arguments.
%
%      MYNOISESUPPRESSIONDEMO('Property','Value',...) creates a new MYNOISESUPPRESSIONDEMO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MyNoiseSuppressionDemo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MyNoiseSuppressionDemo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MyNoiseSuppressionDemo

% Last Modified by GUIDE v2.5 21-Dec-2015 09:28:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MyNoiseSuppressionDemo_OpeningFcn, ...
                   'gui_OutputFcn',  @MyNoiseSuppressionDemo_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MyNoiseSuppressionDemo is made visible.
function MyNoiseSuppressionDemo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MyNoiseSuppressionDemo (see VARARGIN)

% Choose default command line output for MyNoiseSuppressionDemo
handles.output = hObject;


%自己的全局参数初始化
[b, handles.main_data.f_s] = wavread('./signals/white.wav');
[s, handles.main_data.f_s] = wavread('./signals/speech3.wav');
if(length(s)<=length(b))
    handles.main_data.signals.b = b(1:length(s));
    handles.main_data.signals.s = s;
else
    handles.main_data.signals.s = s(1:length(b));
    handles.main_data.signals.b = b;
end

set(handles.pureSignalSampleRate,'String',['纯净语音信号采样率：',num2str(handles.main_data.f_s),' Hz']);


handles.main_data.signals.s      = handles.main_data.signals.s / (max(abs(handles.main_data.signals.s))+eps) * 0.8;%音量归一化处理
handles.main_data.signals.SNR    = 10;
handles.main_data.signals.b      = handles.main_data.signals.b / (std(handles.main_data.signals.b)+eps) * std(handles.main_data.signals.s);
handles.main_data.signals.y      = handles.main_data.signals.s + 10^(-handles.main_data.signals.SNR/20) * handles.main_data.signals.b;
handles.main_data.signals.s_dach = zeros(size(handles.main_data.signals.s));
handles.main_data.N_FFT          = 256;
handles.main_data.frameshift     =  64;
handles.main_data.max_att        =  20;
handles.main_data.overest        =   1;
handles.main_data.signals.SNR_out = 10;

%For SpectrumSub beta;
handles.main_data.beta = 0.002;
%保存加噪语音,for debugging
%wavwrite(handles.main_data.signals.y,handles.main_data.f_s,16,'./withNoise.wav');

handles.spectrumImprove = 1;
handles.visibleDiagram = 1;%是否显示框图；
handles.diagram = imread('./wiener.png');
%初始化显示
handles.methodsChoice = 'Wiener';
% set(handles.Spectrum_improve,'Visible','Off');
load cmap.mat;
colormap(mat0);


handles = ns_start_sim(handles,'Wiener');
set(handles.SNR_out,'String',['SNR: ',num2str(handles.main_data.signals.SNR_out),' dB']);
plotSignals(handles,'s_time');
plotSignals(handles,'s_dach_time');



% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MyNoiseSuppressionDemo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MyNoiseSuppressionDemo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in plot1Selection.
function plot1Selection_Callback(hObject, eventdata, handles)
% hObject    handle to plot1Selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns plot1Selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from plot1Selection
contents = cellstr(get(hObject,'String'));
plotSelection = contents{get(hObject,'Value')};
switch plotSelection
    case 'Plot1:s(n)时域'
        plotSignals(handles,'s_time');
    case 'Plot1:s(n)频域'
        plotSignals(handles,'s_tf');
    case 'Plot1:s(n)+b(n)时域'
        plotSignals(handles,'y_time');
        
    case 'Plot1:s(n)+b(n)频域'
        plotSignals(handles,'y_tf');
end
    


% --- Executes during object creation, after setting all properties.
function plot1Selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plot1Selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in plot2Selection.
function plot2Selection_Callback(hObject, eventdata, handles)
% hObject    handle to plot2Selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns plot2Selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from plot2Selection


contents = cellstr(get(hObject,'String'));
plotSelection = contents{get(hObject,'Value')};
switch plotSelection
    case 'Plot2:^s(n)时域'
        plotSignals(handles,'s_dach_time');
    case 'Plot2:^s(n)频域'
        plotSignals(handles,'s_dach_tf');
end


% --- Executes during object creation, after setting all properties.
function plot2Selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plot2Selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SNR_Callback(hObject, eventdata, handles)
% hObject    handle to SNR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.changeConfirm,'Visible','On');
 get(hObject,'String') 
 handles.main_data.signals.SNR = str2double(get(hObject,'String'));
% 
% handles = ns_start_sim(handles,'SpectrumSub');
% 
 set(handles.SNR_in,'String',['SNR: ',num2str(handles.main_data.signals.SNR),' dB']);
 
 guidata(hObject,handles);
 


% --- Executes during object creation, after setting all properties.
function SNR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SNR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in showSystemDiagram.
function showSystemDiagram_Callback(hObject, eventdata, handles)
% hObject    handle to showSystemDiagram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.visibleDiagram==1)
    
    set(handles.axes3,'Visible','On');
% Hint: get(hObject,'Value') returns toggle state of showSystemDiagram
    axes(handles.axes3);
    imshow(handles.diagram);
    handles.visibleDiagram = 0;
else
    handles.visibleDiagram = 1;
    set(handles.axes3,'Visible','Off');
    axes(handles.axes3);
    cla;
end
guidata(hObject,handles);



function Max_att_Callback(hObject, eventdata, handles)
% hObject    handle to Max_att (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Max_att as text
%        str2double(get(hObject,'String')) returns contents of Max_att as a double

 get(hObject,'String') 
 handles.main_data.max_att = str2double(get(hObject,'String'));
% 
% handles = ns_start_sim(handles,'SpectrumSub');
% 
set(handles.changeConfirm,'Visible','On');
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Max_att_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Max_att (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Overest_Callback(hObject, eventdata, handles)
% hObject    handle to Overest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

 get(hObject,'String') 
 handles.main_data.overest = str2double(get(hObject,'String'));
% 
% handles = ns_start_sim(handles,'SpectrumSub');
% 
set(handles.changeConfirm,'Visible','On');
 guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of Overest as text
%        str2double(get(hObject,'String')) returns contents of Overest as a double


% --- Executes during object creation, after setting all properties.
function Overest_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Overest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in selectWierner.
function selectWierner_Callback(hObject, eventdata, handles)
% hObject    handle to selectWierner (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.showKalmanDemo,'Visible','Off');
set(handles.Max_att,'Enable','On','Visible','On');
set(handles.Overest,'Enable','On','Visible','On');
set(handles.Max_att_text,'Visible','On');
set(handles.Overest_text,'Visible','On');
set(handles.beta,'Visible','Off');
set(handles.beta_edit,'Visible','Off');
% set(handles.Max_att_text,'Visible','On');
% set(handles.Spectrum_improve,'Visible','Off');

handles.diagram = imread('./wiener.png');
handles.methodsChoice = 'Wiener';
if(handles.visibleDiagram ==0)
        axes(handles.axes3);
    imshow(handles.diagram);
end

handles = ns_start_sim(handles,handles.methodsChoice);
% Hint: get(hObject,'Value') returns toggle state of selectWierner
changeConfirm_Callback(hObject, eventdata, handles);

% --- Executes on button press in selectSpectrumSub.
function selectSpectrumSub_Callback(hObject, eventdata, handles)
% hObject    handle to selectSpectrumSub (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.showKalmanDemo,'Visible','Off');
set(handles.Max_att,'Enable','Off');
set(handles.Overest,'Enable','Off','Visible','Off');
set(handles.beta,'Visible','On');
set(handles.beta_edit,'Visible','On');
% set(handles.Max_att_text,'Visible','Off');
% set(handles.Spectrum_improve,'Visible','On');


handles.diagram = imread('./SpectrumSub.png');
if(handles.visibleDiagram ==0)
     axes(handles.axes3);
    imshow(handles.diagram);
end

handles.methodsChoice = 'SpectrumSub';
% Hint: get(hObject,'Value') returns toggle state of selectSpectrumSub
changeConfirm_Callback(hObject, eventdata, handles);





% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1








%自定义函数 
function plotSignals(handles,plot_method)


switch plot_method
    
    case 's_time'                       
        axes(handles.axes1);
        max_abs = max(abs([handles.main_data.signals.s; handles.main_data.signals.y; handles.main_data.signals.s_dach]));
        t = (0:(length(handles.main_data.signals.s)-1)) / handles.main_data.f_s;
        plot(t,handles.main_data.signals.s,'LineWidth',1);
        axis([0 length(handles.main_data.signals.s)/handles.main_data.f_s+eps -1.1*max_abs-eps 1.1*max_abs+eps])                
        handle_1 = line([-1 -1],[-1.2*max_abs-eps 1.2*max_abs+eps]);
        set(handle_1,'Color','k');
        set(handle_1,'LineWidth',2);
        set(handle_1,'EraseMode','xor');                        
        grid on;  
        xlabel('Time in seconds');    
        legend('Undistorted speech signal - s(n)')
        
    case 's_tf'        
        axes(handles.axes1);
        [spec f t] = specgram(handles.main_data.signals.s,512,handles.main_data.f_s,hanning(512),512-64);
        imagesc(t,f/1000,20*log10(abs(spec)+eps));
        axis xy;     
        set(gca,'CLim',[-80 40]);
        handle_1 = line([-1 -1],[0 handles.main_data.f_s]);
        set(handle_1,'Color','k');
        set(handle_1,'LineWidth',2);
        set(handle_1,'EraseMode','xor');                        
        grid on;  
        xlabel('Time in seconds');
        ylabel('Frequency in kHz');
        legend('Undistorted speech signal - s(n)');
        
    case 'y_time'                       
    
        axes(handles.axes1);
        max_abs = max(abs([handles.main_data.signals.s; handles.main_data.signals.y; handles.main_data.signals.s_dach]));
        t = (0:(length(handles.main_data.signals.y)-1)) / handles.main_data.f_s;
        plot(t,handles.main_data.signals.y,'LineWidth',1);
        axis([0 length(handles.main_data.signals.y)/handles.main_data.f_s+eps -1.1*max_abs-eps 1.1*max_abs+eps])                
        handle_1 = line([-1 -1],[-1.2*max_abs-eps 1.2*max_abs+eps]);
        set(handle_1,'Color','k');
        set(handle_1,'LineWidth',2);
        set(handle_1,'EraseMode','xor');                        
        grid on;  
        xlabel('Time in seconds');    
        legend('Microphone signal - y(n)')  
        
    case 'y_tf'        
        axes(handles.axes1)
        [spec f t] = specgram(handles.main_data.signals.y,512,handles.main_data.f_s,hanning(512),512-64);
        imagesc(t,f/1000,20*log10(abs(spec)+eps));
        axis xy;     
        set(gca,'CLim',[-80 40]);
        handle_1 = line([-1 -1],[0 handles.main_data.f_s]);
        set(handle_1,'Color','k');
        set(handle_1,'LineWidth',2);
        set(handle_1,'EraseMode','xor');                        
        grid on;  
        xlabel('Time in seconds');
        ylabel('Frequency in kHz');
        legend('Microphone signal - y(n)');        
      
    case 's_dach_time'                         
        axes(handles.axes2);
        max_abs = max(abs([handles.main_data.signals.s; handles.main_data.signals.y; handles.main_data.signals.s_dach]));
        t = (0:(length(handles.main_data.signals.y)-1)) / handles.main_data.f_s;
        plot(t,handles.main_data.signals.s_dach,'LineWidth',1);
        axis([0 length(handles.main_data.signals.y)/handles.main_data.f_s+eps -1.1*max_abs-eps 1.1*max_abs+eps])                
        handle_1 = line([-1 -1],[-1.2*max_abs-eps 1.2*max_abs+eps]);
        set(handle_1,'Color','k');
        set(handle_1,'LineWidth',2);
        set(handle_1,'EraseMode','xor');                        
        grid on;  
        xlabel('Time in seconds');    
        legend('Output signal');    
        
    case 's_dach_tf'        
        axes(handles.axes2)
        [spec f t] = specgram(handles.main_data.signals.s_dach,512,handles.main_data.f_s,hanning(512),512-64);
        imagesc(t,f/1000,20*log10(abs(spec)+eps));
        axis xy;     
        set(gca,'CLim',[-80 40]);
        handle_1 = line([-1 -1],[0 handles.main_data.f_s]);
        set(handle_1,'Color','k');
        set(handle_1,'LineWidth',2);
        set(handle_1,'EraseMode','xor');                        
        grid on;  
        xlabel('Time in seconds');
        ylabel('Frequency in kHz');
        legend('Output signal');
        colormap;
        
end;

drawnow;



%滤波过程
function return_handles =  ns_start_sim(handles,methods)

return_handles = handles;

switch methods
    case 'Wiener'
        %暂时是维纳滤波

        %**************************************************************************
        % Generate noise with appropriate power
        %**************************************************************************
        return_handles.main_data.signals.b_temp = 10^(-return_handles.main_data.signals.SNR/20) * return_handles.main_data.signals.b;
        return_handles.main_data.signals.y  = return_handles.main_data.signals.s + return_handles.main_data.signals.b_temp;

        %**************************************************************************
        % Window function
        %**************************************************************************
        h_win = hanning(return_handles.main_data.N_FFT,'periodic');
        h_win = h_win / sqrt(sum(h_win.^2) / return_handles.main_data.frameshift);

        %**************************************************************************
        % Spectrogram of the noisy input signal
        %**************************************************************************
        return_handles.main_data.signals.Specgram_y = spectrogram(return_handles.main_data.signals.y,h_win, ...
                                                   return_handles.main_data.N_FFT-return_handles.main_data.frameshift, ...
                                                   return_handles.main_data.N_FFT);

        %**************************************************************************
        % Estimation of power spectral density of the noise
        %**************************************************************************                                     
        return_handles.main_data.signals.Specgram_b_temp =  spectrogram(return_handles.main_data.signals.b_temp,h_win, ...
                                                   return_handles.main_data.N_FFT-return_handles.main_data.frameshift, ...
                                                   return_handles.main_data.N_FFT);       
        S_bb = mean(abs(return_handles.main_data.signals.Specgram_b_temp).^2,2);                                       
        S_bb = S_bb * return_handles.main_data.overest;          

        %**************************************************************************
        % Compute the output spectrogram
        %**************************************************************************
        [N_f, N_t]                        = size(return_handles.main_data.signals.Specgram_y);
        return_handles.main_data.signals.Specgram_s_dach = zeros(size(return_handles.main_data.signals.Specgram_y));

        att_lin = 10^(-return_handles.main_data.max_att/20);
        for k=1:N_t
            H_wiener = max(att_lin, 1 - S_bb./abs(return_handles.main_data.signals.Specgram_y(:,k)).^2);
            return_handles.main_data.signals.Specgram_s_dach(:,k) = return_handles.main_data.signals.Specgram_y(:,k) .* H_wiener;                 
        end;

        %**************************************************************************
        % Compute the output signal
        %**************************************************************************
        return_handles.main_data.signals.s_dach = zeros(size(return_handles.main_data.signals.y));
        for k=1:N_t
           return_handles.main_data.signals.s_dach(1+(k-1)*return_handles.main_data.frameshift:return_handles.main_data.N_FFT+(k-1)*return_handles.main_data.frameshift) = ...
             return_handles.main_data.signals.s_dach(1+(k-1)*return_handles.main_data.frameshift:return_handles.main_data.N_FFT+(k-1)*return_handles.main_data.frameshift) ...
             + real(ifft([return_handles.main_data.signals.Specgram_s_dach(:,k);conj(return_handles.main_data.signals.Specgram_s_dach(end-1:-1:2,k))])) ...
             .* h_win;
        end
    case 'SpectrumSub'
            fs = return_handles.main_data.f_s;            
            return_handles.main_data.signals.b_temp = 10^(-return_handles.main_data.signals.SNR/20) * return_handles.main_data.signals.b;
            return_handles.main_data.signals.y  = return_handles.main_data.signals.s + return_handles.main_data.signals.b_temp;

            x = return_handles.main_data.signals.y;

            len = floor(20*fs/1000);            % Frame size in samples
            if rem(len,2) == 1, len=len+1; end;
            PERC = 50;                          % window overlap in percent of frame size
            len1 = floor(len*PERC/100);
            len2 = len-len1; 

            Thres = 3;      % VAD threshold in dB SNRseg 
            Expnt = 2.0;    % power exponent
            beta = return_handles.main_data.beta;
            G = 0.9;

            win = hamming(len);
            winGain = len2/sum(win); % normalization gain for overlap+add with 50% overlap

            % Noise magnitude calculations - assuming that the first 5 frames is noise/silence
            nFFT = 2*2^nextpow2(len);
            noise_mean = zeros(nFFT,1);
            j=1;
            for k = 1:5
               noise_mean = noise_mean+abs(fft(win.*x(j:j+len-1),nFFT));
               j = j+len;
            end
            noise_mu = noise_mean/5;

            %--- allocate memory and initialize various variables
            k = 1;
            img = sqrt(-1);
            x_old = zeros(len1,1);
            Nframes = floor(length(x)/len2)-1;
            xfinal = zeros(Nframes*len2,1);

            %=========================    Start Processing   ===============================
            for n = 1:Nframes 
                insign = win.*x(k:k+len-1);      % Windowing
                spec = fft(insign,nFFT);         % compute fourier transform of a frame
                sig = abs(spec);                 % compute the magnitude
                %save the noisy phase information 
                theta = angle(spec);  
                SNRseg = 10*log10(norm(sig,2)^2/norm(noise_mu,2)^2);
                if Expnt == 1.0     % 幅度谱
                    alpha = berouti1(SNRseg);
                else
                    alpha = berouti(SNRseg); % 功率谱
                end
                %&&&&&&&&&
                sub_speech = sig.^Expnt - alpha*noise_mu.^Expnt;
                diffw = sub_speech - beta*noise_mu.^Expnt;     % 当纯净信号小于噪声信号的功率时
                % beta negative components
                z = find(diffw <0);  
                if~isempty(z)
                    sub_speech(z) = beta*noise_mu(z).^Expnt;   % 用估计出来的噪声信号表示下限值
                end
                % --- implement a simple VAD detector --------------
                if (SNRseg < Thres)   % Update noise spectrum
                    noise_temp = G*noise_mu.^Expnt+(1-G)*sig.^Expnt;    % 平滑处理噪声功率谱
                    noise_mu = noise_temp.^(1/Expnt);                   % 新的噪声幅度谱
                end
            % flipud函数实现矩阵的上下翻转，是以矩阵的“水平中线”为对称轴
            %交换上下对称元素
                sub_speech(nFFT/2+2:nFFT) = flipud(sub_speech(2:nFFT/2));
                x_phase = (sub_speech.^(1/Expnt)).*(cos(theta)+img*(sin(theta)));
                % take the IFFT 
                xi = real(ifft(x_phase));
                % --- Overlap and add ---------------
                xfinal(k:k+len2-1)=x_old+xi(1:len1);
                x_old = xi(1+len1:len);
                k = k+len2;
            end
            dumpNum = length(x) - length(xfinal);
            xfinal = [xfinal; zeros(dumpNum,1)];
            return_handles.main_data.signals.s_dach = winGain*xfinal;
             % wavwrite(winGain*xfinal,fs,16,'./output.wav');
    case 'KalmanFilter'
                %**************************************************************************
            % Generate noise with appropriate power
            %**************************************************************************
%             return_handles.main_data.signals.b_temp = 10^(-return_handles.main_data.signals.SNR/20) * return_handles.main_data.signals.b;
%             return_handles.main_data.signals.y  = return_handles.main_data.signals.s + return_handles.main_data.signals.b_temp;
% 
% 
%             y = return_handles.main_data.signals.s;
%             y = y';
%             x = return_handles.main_data.signals.y;
%             x = x';
% 
%             %%%%%%%%%%%%%%%原声音信号和加噪声后的信号%%%%%%%%%%%%%%%
%             % figure(1);
%             % subplot(211);plot(m1(1,:),m1(2,:));xlabel('时间');ylabel('幅度');title('原声音信号');
%             % subplot(212);plot(m1(1,:),x);xlabel('时间');ylabel('幅度');title('加噪声后的信号');
% 
%             %%%%%%%%%%%%%%%%%%%%%%%%%输入参数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Fs=return_handles.main_data.f_s;                      %信号采样的频率
% 
%             % wavwrite(x,Fs,'Noisy.wav');
% 
%             bits=16;                 %信号采样的位数
%             N=256;                      %帧长
%             m=N/2;                      %每帧移动的距离
%             lenth=length(x);            %输入信号的长度
%             count=floor(lenth/m)-1;     %处理整个信号需要移动的帧数%%%先不考虑补零的问题
%             p=11;                             %AR模型的阶数
%             a=zeros(1,p);
%             w=hamming(N);                   %加汉明窗函数
%             y_temp=0;
%             F=zeros(11,11);           %转移矩阵
%             F(1,2)=1;
%             F(2,3)=1;
%             F(3,4)=1;
%             F(4,5)=1;
%             F(5,6)=1;
%             F(6,7)=1;
%             F(7,8)=1;
%             F(8,9)=1;
%             F(9,10)=1;
%             F(10,11)=1;
%             H=zeros(1,p);                        %
%             S0=zeros(p,1);
%             P0=zeros(p);
%             S=zeros(p);
%             H(11)=1;
%             s=zeros(N,1);
%             G=H';
%             P=zeros(p);
%             %%%%%%%%%%%%%%%%测试噪声协方差%%%%%%%%%%%%%%%%%%%%%%
%             y_temp=cov(x(1:7680));
%             x_frame=zeros(256,1);
% %             x_frame1=zeros(256,1);
%             T=zeros(lenth,1);
%             for r=1:count
%                 %%%%%%%%%%%%%%%%%%%5%%%%%分帧处理%%%%%%%%%%%%%%%%%%%%%        
%                 x_frame=x((r-1)*m+1:(r+1)*m);          
%                 %%%%%%%%%%%%%%%%采用LPC模型求转移矩阵参数%%%%%%%%%%%%%%   
%                 if r==1
%                     [a,VS]=lpc(x_frame(:),p);  
%                 else
%                     [a,VS]=lpc(T((r-2)*m+1:(r-2)*m+256),p);
%                 end
%                 %%%%%%%%%%%%%%%%帧长内过程噪声协方差%%%%%%%%%%%%%%%%%%
%                 if (VS-y_temp>0)   
%                     VS=VS-y_temp;
%                 else
%                     VS=0.0005;
%                 end
%                 F(p,:)=-1*a(p+1:-1:2);
%                 for j=1:256
%                     if(j==1)
%                         S=F*S0;
%                         Pn=F*P*F'+G*VS*G';
%                     else
%                         S=F*S;      %时间更新方程
%                         Pn=F*P*F'+G*VS*G';
%                     end
%                     K=Pn*H'*(y_temp+H*P*H').^(-1); %卡尔曼增益
%                     P=(eye(p)-K*H)*Pn;                     %测量更新方程
%                     S=S+K*[x_frame(j)-H*S];
%                     T((r-1)*m+j)=H*S;
%                 end
%                 %%%%%%%%%%%%%%%%对得到的每帧数据进行加窗操作%%%%%%%%%%%%%%%%%%%%%%%%
%                 ss(1:256,r)=T((r-1)*m+1:(r-1)*m+256);
%                 sss(1:256,r)=ss(1:256,r).*w;            
%             end
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%合帧操作%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     
%             for r=1:count
%                          
%                 if r==1
%                       
%                     s_out(1:128)=sss(1:128,r);
%                         
%                 else
%                     if r==count
%                         
%                         s_out(r*m+1:r*m+m)=sss(129:256,r);
%                        
%                     else
%                         
%                         s_out(((r-1)*m+1):((r-1)*m+m))=sss(129:256,r-1)+sss(1:128,r);
%                         
%                     end
%                     
%                 end
%                 
%             end
%             
%             return_handles.main_data.signals.s_dach = [s_out,y(length(s_out)+1:end)]';
%%The above version of Kalman filter seems to have bugs. So we use the Kalman filter implementation from KalmanSignalDenoiser.m
          return_handles.main_data.signals.b_temp = 10^(-return_handles.main_data.signals.SNR/20) * return_handles.main_data.signals.b;
          return_handles.main_data.signals.y  = return_handles.main_data.signals.s + return_handles.main_data.signals.b_temp;
%           y_ = return_handles.main_data.signals.y;
          s_out = KalmanSignalDenoiser(return_handles.main_data.signals.y,...
              return_handles.main_data.signals.s,return_handles.main_data.f_s);
          %补上没有处理的最后0.几帧
          return_handles.main_data.signals.s_dach = [s_out,...
              return_handles.main_data.signals.y(length(s_out)+1:end)']';

end
%计算输出信噪比
Ps = norm(return_handles.main_data.signals.s)^2;
Pn = norm(return_handles.main_data.signals.s_dach-return_handles.main_data.signals.s)^2;
return_handles.main_data.signals.SNR_out =  10*log10(Ps/Pn);



% --- Executes on button press in changeConfirm.
function changeConfirm_Callback(hObject, eventdata, handles)
% hObject    handle to changeConfirm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = ns_start_sim(handles,handles.methodsChoice);
set(handles.SNR_out,'String',['SNR: ',num2str(handles.main_data.signals.SNR_out),' dB']);
contents = cellstr(get(handles.plot1Selection,'String'));
plotSelection = contents{get(handles.plot1Selection,'Value')};
switch plotSelection
    case 'Plot1:s(n)时域'
        plotSignals(handles,'s_time');
    case 'Plot1:s(n)频域'
        plotSignals(handles,'s_tf');
    case 'Plot1:s(n)+b(n)时域'
        plotSignals(handles,'y_time');
        
    case 'Plot1:s(n)+b(n)频域'
        plotSignals(handles,'y_tf');
end
contents = cellstr(get(handles.plot2Selection,'String'));
plotSelection = contents{get(handles.plot2Selection,'Value')};
switch plotSelection
    case 'Plot2:^s(n)时域'
        plotSignals(handles,'s_dach_time');
    case 'Plot2:^s(n)频域'
        plotSignals(handles,'s_dach_tf');
end
set(handles.changeConfirm,'Visible','Off');
guidata(hObject,handles);
% kalman_demo_main()



% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in playSound_origin.
function playSound_origin_Callback(hObject, eventdata, handles)
% hObject    handle to playSound_origin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
playSound(handles,1);

% --- Executes on button press in playSound_noise.
function playSound_noise_Callback(hObject, eventdata, handles)
% hObject    handle to playSound_noise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
playSound(handles,2);
% --- Executes on button press in playSound_noise_Suppressed.
function playSound_noise_Suppressed_Callback(hObject, eventdata, handles)
% hObject    handle to playSound_noise_Suppressed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
playSound(handles,3);





 

% --- Executes on button press in Spectrum_improve.
function Spectrum_improve_Callback(hObject, eventdata, handles)
% hObject    handle to Spectrum_improve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.spectrumImprove ==0)
    
    handles.methodsChoice = 'SpectrumSub';
    handles.spectrumImprove = 1;
    changeConfirm_Callback(hObject, eventdata, handles);
else
     handles.methodsChoice = 'SpectrumSub';
    handles.spectrumImprove = 0;
    changeConfirm_Callback(hObject,eventdata,handles);
end

    
% Hint: get(hObject,'Value') returns toggle state of Spectrum_improve
function a = berouti1(SNR)
if SNR >= -5.0 & SNR <= 20
	a = 3-SNR*2/20;
else
    if SNR < -5.0
        a = 4;
    end
    if SNR > 20
        a = 1;
	end
end


function a = berouti(SNR)
if SNR >= -5.0 & SNR <= 20
	a = 4-SNR*3/20; 
else
    if SNR < -5.0
        a = 5;
    end
	if SNR > 20
        a = 1;
    end
end


% --- Executes on selection change in noiseType.
function handles = noiseType_Callback(hObject, eventdata, handles)
% hObject    handle to noiseType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

 contents = cellstr(get(hObject,'String'));
 noiseType = contents{get(hObject,'Value')} ;
 
 
 switch noiseType%噪声信号，大多数从NOISE-92中来
     case 'Turbine'
           [b, f_s] = wavread('./signals/turbine.wav');
     case 'White'
           [b, f_s] = wavread('./signals/white.wav');
     case 'Pink'
           [b, f_s] = wavread('./signals/pink.wav');     
     case 'machinegun'
          [b, f_s] = wavread('./signals/machinegun.wav');
     case 'destroyerengine'
          [b, f_s] = wavread('./signals/destroyerengine.wav');
     case 'volvo'
          [b, f_s] = wavread('./signals/volvo.wav');

 end

 if( f_s  ~= handles.main_data.f_s)%如果读入的噪声信号的原始采样率不等于当前语音信号的采样率
%          wavwrite(b,handles.main_data.f_s,'./Noise_temp.wav');%统一纯净语音和噪音的采样率，一个简单的办法，将噪音信号以纯净信号的采样率为参照，再次采样并保存
%          [b, f_s] = wavread('./Noise_temp.wav');
               b = resample(b,handles.main_data.f_s,f_s);%如果读入的噪声信号的原始采样率不等于当前语音信号的采样率，将噪声重采样。
                
 end
 
if(length(handles.main_data.signals.s)<=length(b))
                    handles.main_data.signals.b = b(1:length(handles.main_data.signals.s));
       
else
                    s = (handles.main_data.signals.s);
                    handles.main_data.signals.s = s(1:length(b));
                    handles.main_data.signals.b = b;

          
end
handles.main_data.signals.b      = handles.main_data.signals.b / (std(handles.main_data.signals.b)+eps) * std(handles.main_data.signals.s);

set(handles.changeConfirm,'Visible','On');
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function noiseType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noiseType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function playSound(handles,selection)
h1 = handles.axes1;
set(gcf,'CurrentAxes',h1);
hold on;
c1 = plot([0 0],[0 0],'-k','LineWidth',2);
h2 = handles.axes2;
set(gcf,'CurrentAxes',h2);
hold on;
c2 = plot([0 0],[0 0],'-k','LineWidth',2);
len_sec = length(handles.main_data.signals.s_dach) / handles.main_data.f_s;
switch selection
    case 1
        soundsc(handles.main_data.signals.s,handles.main_data.f_s);
    case 2
        soundsc(handles.main_data.signals.y,handles.main_data.f_s);
    case 3
        soundsc(handles.main_data.signals.s_dach,handles.main_data.f_s);
end

tic;
while(toc<len_sec)
    t  = toc;
     set(gcf,'CurrentAxes',h1); 
    set(c1,'XData',[t t],'YData',ylim);
    set(gcf,'CurrentAxes',h2); 
    set(c2,'XData',[t t],'YData',ylim);   
    drawnow;
end
delete(c1);
delete(c2);
axes(h1);hold off;
axes(h2);hold off;



function beta_edit_Callback(hObject, eventdata, handles)
% hObject    handle to beta_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of beta_edit as text
%        str2double(get(hObject,'String')) returns contents of beta_edit as a double
[beta,OK] = str2num(get(hObject,'String'));
set(handles.changeConfirm,'Visible','On');
if(OK ~=0)
    handles.main_data.beta = beta;
    guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function beta_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to beta_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in selectInputSignal.
function selectInputSignal_Callback(hObject, eventdata, handles)
% hObject    handle to selectInputSignal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile({'*.wav','(*.wav)'},'选择纯净语音信号');

if(filename == 0)
   warndlg('You did not select any input file!','warning','modal');
    return;
end
[s,f_s] = wavread([pathname,filename]);
handles.main_data.signals.s = s;
if(handles.main_data.f_s ~= f_s)%如果采样率没变，不用理会
    handles.main_data.f_s = f_s;%否则更新采样率，同时噪声也应该更新，因为采样率变了
    handles = noiseType_Callback(handles.noiseType, eventdata, handles);%调用读取噪声的回调函数即可更新；
end
set(handles.pureSignalSampleRate,'String',['纯净语音信号采样率：',num2str(f_s),' Hz']);
set(handles.changeConfirm,'Visible','On');
%更新Plot1的显示
contents = cellstr(get(handles.plot1Selection,'String'));
plotSelection = contents{get(handles.plot1Selection,'Value')};
switch plotSelection
    case 'Plot1:s(n)时域'
        plotSignals(handles,'s_time');
    case 'Plot1:s(n)频域'
        plotSignals(handles,'s_tf');
    case 'Plot1:s(n)+b(n)时域'
        plotSignals(handles,'y_time');
        
    case 'Plot1:s(n)+b(n)频域'
        plotSignals(handles,'y_tf');
end


guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function changeConfirm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to changeConfirm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in KalmanFilter.
function KalmanFilter_Callback(hObject, eventdata, handles)
% hObject    handle to KalmanFilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.methodsChoice = 'KalmanFilter';
set(handles.Overest,'Visible','Off');
set(handles.Overest_text,'Visible','Off');
set(handles.Max_att,'Visible','Off');


set(handles.Max_att_text,'Visible','Off');
set(handles.beta,'Visible','Off');
set(handles.beta_edit,'Visible','Off');
handles.diagram = imread('./kalmanfilter.png');
if(handles.visibleDiagram ==0)
        axes(handles.axes3);
    imshow(handles.diagram);
end
% handles = ns_start_sim(handles,handles.methodsChoice);
% Hint: get(hObject,'Value') returns toggle state of KalmanFilter
changeConfirm_Callback(hObject, eventdata, handles);
set(handles.showKalmanDemo,'Visible','On');


% --- Executes on button press in showKalmanDemo.
function showKalmanDemo_Callback(hObject, eventdata, handles)
% hObject    handle to showKalmanDemo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%set(handles.uipanel1,'Visible','Off');
kalman_demo();
% set(handles.uipanel1,'Visible','Off');


% --- Executes on button press in SpeechEvaluate.
function SpeechEvaluate_Callback(hObject, eventdata, handles)
% hObject    handle to SpeechEvaluate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(isempty(handles.main_data.signals.s))
    return
else
    %Generate temp wave for pure speech and noisy speech
    %because the evaluation must be performed on a sample rate of either
    %8000 or 16000. Resample the signals.
    temp_pure = resample(handles.main_data.signals.s,16000,handles.main_data.f_s);
    wavwrite(temp_pure,16000,'signals\temp_pure_signal.wav');
    temp_noisy = resample(handles.main_data.signals.s_dach,16000,handles.main_data.f_s);
    wavwrite(temp_noisy,16000,'signals\temp_pure_noisy.wav');
    
    [pesq_mos,mos_lqo,mos_lqo_wb] = test_pesq2_mtlb('signals','temp_pure_signal.wav','temp_pure_noisy.wav',16000);
    
    set(handles.PESQ_MOS,'String',['PESQ MOS: ',num2str(pesq_mos)]);
    set(handles.MOS_LQO,'String',['MOS LQO: ',num2str(mos_lqo)]);
    set(handles.MOS_LQO_WB,'String',['MOS LQO: ',num2str(mos_lqo_wb)]);

    delete('./signals/temp_pure_signal.wav');
    delete('./signals/temp_pure_noisy.wav');
    
end
