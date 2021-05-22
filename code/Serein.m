function varargout = Serein(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Serein_OpeningFcn, ...
                   'gui_OutputFcn',  @Serein_OutputFcn, ...
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

function Serein_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);

global flag;
global row;
global col;
global resizemethod;
global adjustFlag;
global sizeFlag;
global fixFlag;
global paramFlag;
global frameFlag;
global wordFlag;
global shot;
shot=1;
wordFlag=1;
frameFlag=1;
paramFlag=1;
fixFlag=1;
sizeFlag=1;
adjustFlag=1;
resizemethod=1;
row='0';
col='0';
flag=0;

Area1=imread('area1.png');
Area2=imread('area2.png');
axes(handles.axes1);
imshow(Area1);
axes(handles.axes2);
imshow(Area2);

Open = imread('open.jpg');   
set(handles.OpenButton,'CData',Open);
guidata(hObject, handles);

Save = imread('save.jpg');   
set(handles.SaveButton,'CData',Save);
guidata(hObject, handles);

Crop = imread('crop.jpg');   
set(handles.CropButton,'CData',Crop);
guidata(hObject, handles);

Reset = imread('reset.jpg');   
set(handles.ResetButton,'CData',Reset);
guidata(hObject, handles);

Adjust = imread('adjust.jpg');   
set(handles.AdjustButton,'CData',Adjust);
guidata(hObject, handles);

SizeLogo = imread('size.jpg');   
set(handles.SizeButton,'CData',SizeLogo);
guidata(hObject, handles);

Confirm = imread('confirm.jpg');   
set(handles.ResizeButton,'CData',Confirm);
guidata(hObject, handles);

Fix = imread('fix.jpg');   
set(handles.FixButton,'CData',Fix);
guidata(hObject, handles);

Histogramstatics = imread('HistogramStatics.jpg');   
set(handles.HistogramStaticsButton,'CData',Histogramstatics);
guidata(hObject, handles);

Explosurecorrect = imread('ExplosureCorrect.jpg');   
set(handles.ExplosureCorrect,'CData',Explosurecorrect);
guidata(hObject, handles);

Histogramequalization = imread('HistogramEqualization.jpg');   
set(handles.HistogramEqualizationButton,'CData',Histogramequalization);
guidata(hObject, handles);

Param = imread('param.jpg');   
set(handles.ParamsButton,'CData',Param);
guidata(hObject, handles);

Frame = imread('frame.jpg');   
set(handles.FrameButton,'CData',Frame);
guidata(hObject, handles);

Cloud = imread('Frame1.png'); 
Cloud=imresize(Cloud,[100,100],'nearest');
set(handles.Frame1Button,'CData',Cloud);
guidata(hObject, handles);

Heart = imread('Frame2.png'); 
Heart=imresize(Heart,[100,100],'nearest');
set(handles.Frame2Button,'CData',Heart);
guidata(hObject, handles);

Sign = imread('Frame3.png'); 
Sign=imresize(Sign,[100,100],'nearest');
set(handles.Frame3Button,'CData',Sign);
guidata(hObject, handles);

Star = imread('Frame4.png'); 
Star=imresize(Star,[100,100],'nearest');
set(handles.Frame4Button,'CData',Star);
guidata(hObject, handles);

Rabbit = imread('Frame5.png'); 
Rabbit=imresize(Rabbit,[100,100],'nearest');
set(handles.Frame5Button,'CData',Rabbit);
guidata(hObject, handles);

Word = imread('word.jpg'); 
set(handles.WordButton,'CData',Word);
guidata(hObject, handles);

Write = imread('write.jpg'); 
set(handles.WriteButton,'CData',Write);
guidata(hObject, handles);

Clear = imread('clear.jpg'); 
set(handles.ClearButton,'CData',Clear);
guidata(hObject, handles);

Quit = imread('quit.jpg'); 
set(handles.QuitButton,'CData',Quit);
guidata(hObject, handles);

Help = imread('help.jpg'); 
set(handles.HelpButton,'CData',Help);
guidata(hObject, handles);

Photo = imread('photo.jpg'); 
set(handles.PhotoButton,'CData',Photo);
guidata(hObject, handles);

function varargout = Serein_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


function figure1_CreateFcn(hObject, eventdata, handles)
Background=axes('units','normalized','pos',[0 0 1 1]);
uistack(Background,'bottom');
img=imread('Background.png');
image(img);
colormap gray
set(Background,'handlevisibility','off','visible','off');


function axes1_CreateFcn(hObject, eventdata, handles)
% 将坐标轴和坐标刻度转为白色
set(gca,'XColor',get(gca,'Color')) ;
set(gca,'YColor',get(gca,'Color'));
% 这两行代码功能：去除坐标刻度
set(gca,'XTickLabel',[]); 
set(gca,'YTickLabel',[]);

function axes2_CreateFcn(hObject, eventdata, handles)
% 将坐标轴和坐标刻度转为白色
set(gca,'XColor',get(gca,'Color')) ;
set(gca,'YColor',get(gca,'Color'));
% 这两行代码功能：去除坐标刻度
set(gca,'XTickLabel',[]); 
set(gca,'YTickLabel',[]);

%图像的读取和读取路径显示
function OpenButton_Callback(hObject, eventdata, handles)
global flag;
[FileName,PathName]=uigetfile({'*.bmp;*.jpg;*.png;*.jpeg;*.tif'},'选择一个图片');
if isequal(FileName,0)||isequal(PathName,0) 
    errordlg('请选择文件','提示','modal')
else
    Image=imread([PathName,FileName]);
    handles.Img=Image;
    guidata(hObject,handles);
    [H,W,D]=size(Image);
    flag=1;
    handles.Origin=Image;
    guidata(hObject,handles);
    set(handles.OpenPath,'string',[PathName,FileName]);
    set(handles.ColParam,'String',[num2str(W),'像素']);
    set(handles.RowParam,'String',[num2str(H),'像素']);
    set(handles.DimParam,'String',[num2str(D),'通道']);
    axes(handles.axes1);
    imshow(Image);
    axes(handles.axes2);
    imshow(Image);
end

%图像的保存和保存路径显示
function SaveButton_Callback(hObject, eventdata, handles)
Picture=handles.Img;
[FileName,PathName,FilterIndex]=uiputfile({'*.bmp;*.jpg;*.png;*.jpeg;*.tif'},'保存文件','new_figure');
if FilterIndex ==1
    set(handles.SavePath,'string',[PathName,FileName]);
    imwrite(Picture,[PathName,FileName]);
else
    errordlg('未选择文件保存','提示','modal')
end

function OpenPath_Callback(hObject, eventdata, handles)

function OpenPath_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function SavePath_Callback(hObject, eventdata, handles)

function SavePath_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%亮度调整
function Brightness_Callback(hObject, eventdata, handles)
global flag;
Pos=get(hObject,'Value');
if flag==1
    Image1=handles.Origin;
    ImageSize=size(Image1);
    Dim=ImageSize(3);
    if Dim==3
        HSV=rgb2hsv(Image1);
        HSV(:,:,3)=(Pos+1)*HSV(:,:,3);
        %{
        for i=1:ImageSize(1)
            for j=1:ImageSize(2)
                HSV(i,j,3)=(Pos+1)*HSV(i,j,3);
            end
        end
        %}
        Image2=hsv2rgb(HSV);
        %{
        HSI=rgb2hsi(Image1);
        I=HSI(:,:,3);
        I=I+Pos*0.8;
        HSI(:,:,3)=I;
        Image2=im2uint8(hsi2rgb(HSI));
        %}
    else
        Image2=im2uint8(Image1+pos*128);
    end
    axes(handles.axes2);
    imshow(Image2);
    handles.Img=Image2;
    guidata(hObject,handles);
else
    errordlg('未选择文件','提示','modal')
end
 
function hsi = rgb2hsi(rgb)
rgb = im2double(rgb);
r = rgb(:, :, 1);
g = rgb(:, :, 2);
b = rgb(:, :, 3);

num = 0.5*((r - g) + (r - b));
den = sqrt((r - g).^2 + (r - b).*(g - b));
theta = acos(num./(den + eps));
H = theta;
H(b > g) = 2*pi - H(b > g);
H = H/(2*pi);

num = min(min(r, g), b);
den = r + g + b;
S = 1 - 3.* num./(den + eps);

I = (r + g + b)/3;

hsi = cat(3, H, S, I);

function rgb = hsi2rgb(hsi)
H = hsi(:, :, 1) * 2 * pi;
S = hsi(:, :, 2);
I = hsi(:, :, 3);

R = zeros(size(hsi, 1), size(hsi, 2));
G = zeros(size(hsi, 1), size(hsi, 2));
B = zeros(size(hsi, 1), size(hsi, 2));

idx = find( (0 <= H) & (H < 2*pi/3));
B(idx) = I(idx) .* (1 - S(idx));
R(idx) = I(idx) .* (1 + S(idx) .* cos(H(idx)) ./cos(pi/3 - H(idx)));
G(idx) = 3*I(idx) - (R(idx) + B(idx));

idx = find( (2*pi/3 <= H) & (H < 4*pi/3) );
R(idx) = I(idx) .* (1 - S(idx));
G(idx) = I(idx) .* (1 + S(idx) .* cos(H(idx) - 2*pi/3) ./cos(pi - H(idx)));
B(idx) = 3*I(idx) - (R(idx) + G(idx));

idx = find( (4*pi/3 <= H) & (H <= 2*pi));
G(idx) = I(idx) .* (1 - S(idx));
B(idx) = I(idx) .* (1 + S(idx) .* cos(H(idx) - 4*pi/3) ./cos(5*pi/3 - H(idx)));
R(idx) = 3*I(idx) - (G(idx) + B(idx));

rgb = cat(3, R, G, B);
rgb = im2uint8(rgb);

function Brightness_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%对比度调整
function Contrast_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
Pos=get(hObject,'Value');
if flag==1
    Image3=handles.Img;
    Image4=imadjust(Image3,[],[],5/Pos - 0.4);
    handles.Img=Image4;
    guidata(hObject,handles);
    imshow(Image4);
else
    errordlg('未选择文件','提示','modal');
end    

function Contrast_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%图像锐化
function Sharpen_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
Pos=get(hObject,'Value');
if flag==1
    Image10=handles.Origin;
    I=im2double(Image10);
    [m,n,c]=size(I);
    A=zeros(m,n,c);
    if c==3

for i=2:m-1
    for j=2:n-1
        A(i,j,1)=(I(i+1,j+1,1)+Pos*I(i+1,j,1)+I(i+1,j-1,1)-I(i-1,j+1,1)-Pos*I(i-1,j,1)-I(i-1,j-1,1));
    end
end

for i=2:m-1
    for j=2:n-1
        A(i,j,2)=(I(i+1,j+1,2)+Pos*I(i+1,j,2)+I(i+1,j-1,2)-I(i-1,j+1,2)-Pos*I(i-1,j,2)-I(i-1,j-1,2));
    end
end

for i=2:m-1
    for j=2:n-1
        A(i,j,3)=(I(i+1,j+1,3)+Pos*I(i+1,j,3)+I(i+1,j-1,3)-I(i-1,j+1,3)-Pos*I(i-1,j,3)-I(i-1,j-1,3));
    end
end
if Pos==0
    B=I;
    imshow(B);
else
B=I-A;
imshow(B);
end
    end
    handles.Img=B;
    guidata(hObject,handles);
else
    errordlg('未选择文件','提示','modal');
end


function Sharpen_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%色温调节
function ColorTemp_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
Pos=get(hObject,'Value');
if flag==1
    Image11=handles.Origin;
    [w,h,d]=size(Image11);
    if d==3
    I=im2double(Image11);
    I1=I;
    R=I1(:,:,1);G=I1(:,:,2);B=I1(:,:,3);
    r=R;g=G;b=B*Pos;
    rgb=cat(3,r,g,b);
    imshow(rgb);
    handles.Img=rgb;
    guidata(hObject,handles);
    end
else
    errordlg('未选择文件','提示','modal');
end


function ColorTemp_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%截图按钮
function CropButton_Callback(hObject, eventdata, handles)
global flag;
if flag==1
   Image5=handles.Img;
   ROI=imcrop(Image5);
   axes(handles.axes2);
   imshow(ROI);
   handles.Img=ROI;
   guidata(hObject,handles);  
else
   errordlg('未选择文件','提示','modal');
end

%重置图像按钮
function ResetButton_Callback(hObject, eventdata, handles)
global flag;
set(handles.Brightness,'Value',0);
set(handles.Contrast,'Value',5);
set(handles.Beauty,'Value',0);
set(handles.Sharpen,'Value',0);
set(handles.ColorTemp,'Value',1);
axes(handles.axes2);
if flag==1
    handles.Img=handles.Origin;
    guidata(hObject,handles);
    imshow(handles.Img);
else
    errordlg('未选择文件','提示','modal'); 
end

%修改尺寸按钮
function ResizeButton_Callback(hObject, eventdata, handles)   
global row;
global col;
global flag;
global resizemethod;
axes(handles.axes2);
if flag==1
    Image6=handles.Img;
    if resizemethod==1
        errordlg('请选择插值方法','提示','modal');
    end
    if str2num(row)==0 && str2num(col)==0
        errordlg('请输入图像尺寸','提示','modal');
    elseif str2num(row)==0 && str2num(col)~=0
        errordlg('请输入图像像素高度','提示','modal');
    elseif str2num(row)~=0 && str2num(col)==0
        errordlg('请输入图像像素宽度','提示','modal');
    else
        if resizemethod==2
            handles.Img=imresize(Image6,[str2num(row),str2num(col)],'nearest');
            guidata(hObject,handles);
            imshow(handles.Img);
        elseif resizemethod==3
            handles.Img=imresize(Image6,[str2num(row),str2num(col)],'bilinear');
            guidata(hObject,handles);
            imshow(handles.Img);
        else
            handles.Img=imresize(Image6,[str2num(row),str2num(col)],'bicubic');
            guidata(hObject,handles);
            imshow(handles.Img);
        end
    end      
else
    errordlg('未选择文件','提示','modal');
end
    

%输入修改尺寸的像素宽度
function Width_Callback(hObject, eventdata, handles)
global row;
row=get(hObject,'String');

function Width_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%输入修改尺寸的像素高度
function Height_Callback(hObject, eventdata, handles)
global col;
col=get(hObject,'String');

function Height_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%选择修改尺寸的插值方法
function ResizeMethod_Callback(hObject, eventdata, handles)
global resizemethod;
resizemethod=get(hObject,'Value');

function ResizeMethod_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%直方图统计按钮
function HistogramStaticsButton_Callback(hObject, eventdata, handles)
global flag;
if flag==1
    Size=size(handles.Img);
    Dim=Size(3);
    if Dim==3
        figure(1)
        imhist(rgb2gray(handles.Img));
    else
        figure(1)
        imhist(handles.Img);
    end
else
    errordlg('未选择文件','提示','modal');
end

%直方图均衡化按钮
function HistogramEqualizationButton_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Size=size(handles.Img);
    Dim=Size(3);
    if Dim==3
        imshow(histeq(rgb2gray(handles.Img)));
    else
        imshow(histeq(handles.Img));
    end
else
    errordlg('未选择文件','提示','modal');
    
end

%修复过度曝光按钮
function ExplosureCorrect_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Image7=handles.Origin;
[w,h,d]=size(Image7);
Img_Gray=rgb2gray(Image7);
Mean=mean(Img_Gray(:));
Gamma_val=log10(0.5)/log10(Mean/255);
Gamma_table=zeros(1,256);
for x=1:256
    Gamma_table(x)=(((x-1)/255)^Gamma_val)*255;
end
Gamma_table=round(Gamma_table);
 for i=1:w
     for j=1:h
         Image7(i,j,1)=Gamma_table(Image7(i,j,1)+1);
         Image7(i,j,2)=Gamma_table(Image7(i,j,2)+1);
         Image7(i,j,3)=Gamma_table(Image7(i,j,3)+1);
     end
 end
 imshow(Image7);
 handles.Img=Image7;
 guidata(hObject,handles);
else
    errordlg('未选择文件','提示','modal');
end

%美颜功能
function Beauty_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);

if flag==1
    Image8=handles.Origin;
 pos=round(get(hObject,'Value'));
 Img=Image8;
[m,n,d]=size(Img);
tempsize=5;
sigma0=pos;
sigma1=0.04;
img=double(padarray(Img,[tempsize,tempsize],0))/255;

r = img(:,:,1);
g = img(:,:,2);
b = img(:,:,3);

gauss = fspecial('gauss',2*tempsize+1,sigma0);
[m,n,d] = size(r);
for i = 1+ tempsize : m - tempsize
    for j = 1+ tempsize : n - tempsize
       temp = abs(r(i - tempsize:i + tempsize,j - tempsize:j + tempsize) - r(i,j));
       temp = exp(-temp.^2/(2*sigma1^2));
       filter = gauss.*temp;
       filter = filter/sum(filter(:));
       r(i,j) = sum(sum((r(i - tempsize:i + tempsize,j - tempsize:j + tempsize).*filter)));
    end
end      
img(:,:,1) = r;

gauss = fspecial('gauss',2*tempsize+1,sigma0);
[m,n,d] = size(g);
for i = 1+ tempsize : m - tempsize
    for j = 1+ tempsize : n - tempsize
       temp = abs(g(i - tempsize:i + tempsize,j - tempsize:j + tempsize) - g(i,j));
       temp = exp(-temp.^2/(2*sigma1^2));
       filter = gauss.*temp;
       filter = filter/sum(filter(:));
       g(i,j) = sum(sum((g(i - tempsize:i + tempsize,j - tempsize:j + tempsize).*filter)));
    end
end      
img(:,:,2) = g;

gauss = fspecial('gauss',2*tempsize+1,sigma0);
[m,n,d] = size(b);
for i = 1+ tempsize : m - tempsize
    for j = 1+ tempsize : n - tempsize
       temp = abs(b(i - tempsize:i + tempsize,j - tempsize:j + tempsize) - b(i,j));
       temp = exp(-temp.^2/(2*sigma1^2));
       filter = gauss.*temp;
       filter = filter/sum(filter(:));
       b(i,j) = sum(sum((b(i - tempsize:i + tempsize,j - tempsize:j + tempsize).*filter)));
    end
end      
img(:,:,3) = b;


imshow(img(tempsize+1:m-tempsize,tempsize+1:n-tempsize,:))
else
    errordlg('未选择文件','提示','modal');
end

function Beauty_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%{照片边框特效
%{
function PictureFrame_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
Frame=get(hObject,'Value');
if flag==1
    Image9=handles.Origin;
    if Frame==2
        frame1=imread('Frame1.png');
        frame1=AttachFrame_Black(frame1,Image9);
        handles.Img=frame1;
        guidata(hObject,handles);
        imshow(frame1);
    end
    if Frame==3
        frame2=imread('Frame2.png');
        frame2=AttachFrame_Black(frame2,Image9);
        handles.Img=frame2;
        guidata(hObject,handles);
        imshow(frame2);
    end 
    if Frame==4
        frame3=imread('Frame3.png');
        frame3=AttachFrame_White(frame3,Image9);
        handles.Img=frame3;
        guidata(hObject,handles);
        imshow(frame3);
    end
    if Frame==5
        frame4=imread('Frame4.png');
        frame4=AttachFrame_White(frame4,Image9);
        handles.Img=frame4;
        guidata(hObject,handles);
        imshow(frame4);
    end
    if Frame==6
        frame5=imread('Frame5.png');
        frame5=AttachFrame_White(frame5,Image9);
        handles.Img=frame5;
        guidata(hObject,handles);
        imshow(frame5);
    end
else
    errordlg('未选择文件','提示','modal');
end
%}
                    
function Output=AttachFrame_Black(frame,image)
[rows,cols,dims]=size(image);
frame=imresize(frame,[rows,cols],'nearest');
for i=1:rows
    for j=1:cols
         if frame(i,j,1)==0 && frame(i,j,2)==0 && frame(i,j,3)==0
            frame(i,j,1)=image(i,j,1);
            frame(i,j,2)=image(i,j,2);
            frame(i,j,3)=image(i,j,3);
         end
    end
end
Output=frame;

function Output=AttachFrame_White(frame,image)
[rows,cols,dims]=size(image);
frame=imresize(frame,[rows,cols],'nearest');
for i=1:rows
    for j=1:cols
         if frame(i,j,1)==255 && frame(i,j,2)==255 && frame(i,j,3)==255
            frame(i,j,1)=image(i,j,1);
            frame(i,j,2)=image(i,j,2);
            frame(i,j,3)=image(i,j,3);
         end
    end
end
Output=frame;

function PictureFrame_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%弹出调节菜单栏
function AdjustButton_Callback(hObject, eventdata, handles)
global adjustFlag;
adjustFlag=adjustFlag*-1;
if adjustFlag==-1
    set(hObject,'Position',[950,550,50,100])
    set(handles.Adjust,'Position',[1000,450,200,250])
else
    set(hObject,'Position',[1140,550,50,100])
    set(handles.Adjust,'Position',[1190,450,200,250])    

end

%弹出修改尺寸菜单栏
function SizeButton_Callback(hObject, eventdata, handles)
global sizeFlag;
sizeFlag=sizeFlag*-1;
if sizeFlag==-1
    set(hObject,'Position',[950,300,50,100])
    set(handles.Size,'Position',[1000,250,200,200])
else
    set(hObject,'Position',[1140,300,50,100])
    set(handles.Size,'Position',[1190,250,200,200])    

end

%弹出修复图片菜单栏
function FixButton_Callback(hObject, eventdata, handles)
global fixFlag;
fixFlag=fixFlag*-1;
if fixFlag==-1
    set(hObject,'Position',[950,50,50,100])
    set(handles.Fix,'Position',[1000,0,200,250])
else
    set(hObject,'Position',[1140,50,50,100])
    set(handles.Fix,'Position',[1190,0,200,250])    

end

%弹出图像详细参数菜单栏
function ParamsButton_Callback(hObject, eventdata, handles)
global paramFlag;
global frameFlag;
global wordFlag;
paramFlag=paramFlag*-1;
if paramFlag==-1 || frameFlag==-1 || wordFlag==-1
    set(handles.WordButton,'Position',[400,100,100,40]);
end
if paramFlag==-1
    set(hObject,'Position',[100,100,100,40])
    set(handles.WordButton,'Position',[400,100,100,40])
    set(handles.Param,'Position',[0,0,800,100])
else
    set(hObject,'Position',[100,10,100,40])
    if frameFlag==1 && wordFlag~=-1
    set(handles.WordButton,'Position',[400,10,100,40])
    end
    set(handles.Param,'Position',[0,-90,800,100])    

end

%弹出边框特效菜单栏
function FrameButton_Callback(hObject, eventdata, handles)
global paramFlag;
global frameFlag;
global wordFlag;
frameFlag=frameFlag*-1;
if paramFlag==-1 || frameFlag==-1 || wordFlag==-1
    set(handles.WordButton,'Position',[400,100,100,40]);
end
if frameFlag==-1
    set(hObject,'Position',[800,100,100,40])
    set(handles.WordButton,'Position',[400,100,100,40])
    set(handles.Frame,'Position',[0,0,900,100])
else
    set(hObject,'Position',[800,10,100,40])
    if paramFlag==1 && wordFlag~=-1
    set(handles.WordButton,'Position',[400,10,100,40])
    end
    set(handles.Frame,'Position',[0,-90,900,100])    
end

%相框特效
function Frame1Button_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Image9=handles.Origin;
    frame1=imread('Frame1.png');
    frame1=AttachFrame_Black(frame1,Image9);
    handles.Img=frame1;
    guidata(hObject,handles);
    imshow(frame1);    
else
    errordlg('未选择文件','提示','modal');
end

function Frame2Button_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Image9=handles.Origin;
    frame2=imread('Frame2.png');
    frame2=AttachFrame_Black(frame2,Image9);
    handles.Img=frame2;
    guidata(hObject,handles);
    imshow(frame2);    
else
    errordlg('未选择文件','提示','modal');
end

function Frame3Button_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Image9=handles.Origin;
    frame3=imread('Frame3.png');
    frame3=AttachFrame_White(frame3,Image9);
    handles.Img=frame3;
    guidata(hObject,handles);
    imshow(frame3);    
else
    errordlg('未选择文件','提示','modal');
end

function Frame4Button_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Image9=handles.Origin;
    frame4=imread('Frame4.png');
    frame4=AttachFrame_White(frame4,Image9);
    handles.Img=frame4;
    guidata(hObject,handles);
    imshow(frame4);    
else
    errordlg('未选择文件','提示','modal');
end

function Frame5Button_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    Image9=handles.Origin;
    frame5=imread('Frame5.png');
    frame5=AttachFrame_White(frame5,Image9);
    handles.Img=frame5;
    guidata(hObject,handles);
    imshow(frame5);    
else
    errordlg('未选择文件','提示','modal');
end

%添加文字
function WordButton_Callback(hObject, eventdata, handles)
global paramFlag;
global frameFlag;
global wordFlag;
wordFlag=wordFlag*-1;
if paramFlag==-1 || frameFlag==-1 || wordFlag==-1
    set(handles.WordButton,'Position',[400,100,100,40]);
end
if wordFlag==-1
    set(hObject,'Position',[400,100,100,40])
    set(handles.Word,'Position',[300,0,300,100])
else
    if paramFlag==1 && frameFlag==1
    set(hObject,'Position',[400,10,100,40])
    end
    set(handles.Word,'Position',[300,-90,300,100])    
end

%添加文字按钮
function WriteButton_Callback(hObject, eventdata, handles)
global flag;
axes(handles.axes2);
if flag==1
    wordColor=get(handles.WordColor,'Value');
    if wordColor==1
        Color='k';
    end
    if wordColor==2
        Color='r';
    end
    if wordColor==3
        Color='b';
    end
    if wordColor==4
        Color='g';
    end
    if wordColor==5
        Color='y';
    end 
    
    wordSize=get(handles.WordSize,'String');
    Size=str2num(wordSize);
    if Size==0
        errordlg('请选择字体大小','提示','modal');
    else
    String=inputdlg('请输入文字','提示');
    gtext(String{1,1},'color',Color,'Fontsize',Size);
    end
else
    errordlg('未选择文件','提示','modal');
end

function WordColor_Callback(hObject, eventdata, handles)

function WordColor_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function WordSize_Callback(hObject, eventdata, handles)

function WordSize_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%清楚按钮
function ClearButton_Callback(hObject, eventdata, handles)
Area1=imread('area1.png');
Area2=imread('area2.png');
axes(handles.axes2);
imshow(Area2);
axes(handles.axes1);
imshow(Area1);

%退出按钮
function QuitButton_Callback(hObject, eventdata, handles)
close(Serein)

%打开说明文档
function HelpButton_Callback(hObject, eventdata, handles)
open('使用说明.pdf')

%拍照
function PhotoButton_Callback(hObject, eventdata, handles)
global vid
global shot;
shot=shot*-1;
if shot==-1
vid = videoinput('winvideo',2);
set(vid,'FramesPerTrigger',1);
set(vid,'TriggerRepeat',Inf);
vidRes = get(vid,'VideoResolution');
nBands = get(vid,'NumberOfBands');
axes(handles.axes1);
global hImage
hImage = image(zeros(vidRes(2),vidRes(1),nBands));
preview(vid,hImage);
else
   frame = getsnapshot(vid);
   axes(handles.axes2); 
   imshow(frame);
   handles.Img=frame;
   guidata(hObject,handles);
   closepreview
end



