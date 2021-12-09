#pragma once
// CHANGE THIS TO YOUR PATH
#define FONT_PATH "Maziro.ttf"
// WIN32 boilerplate
#include <windows.h>
#include <commctrl.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <wchar.h>
#include <math.h>
// enable patched d2d1.h interfaces 
#define WIDL_EXPLICIT_AGGREGATE_RETURNS
// have to use the patched version
#include <d2d1.h>
#include <d2d1helper.h>
#include <dwrite.h>
#include <wincodec.h>


#ifndef Assert
#if defined( DEBUG ) || defined( _DEBUG )
#define Assert(b) do {if (!(b)) {OutputDebugStringA("Assert: " #b "\n");}} while(0)
#else
#define Assert(b)
#endif //DEBUG || _DEBUG
#endif

#ifndef HINST_THISCOMPONENT
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#define HINST_THISCOMPONENT ((HINSTANCE)&__ImageBase)
#endif
// End WIN32 boilerplate

#include <gfx_cpp14.hpp>
// include our driver
#include "windows/drivers/dxd2d.hpp"
#include "../fonts/Bm437_ATI_9x16.h"
using namespace gfx;
using namespace windows;

class demo
{
    HWND m_hwnd;
    bool m_done;
    void* m_frame_buffer;
    ID2D1Factory* m_pDirect2dFactory;
    ID2D1HwndRenderTarget* m_pRenderTarget;
    ID2D1SolidColorBrush* m_pLightSlateGrayBrush;
    ID2D1SolidColorBrush* m_pCornflowerBlueBrush;
    int m_render_state;
    int m_render_i;
    // this is your driver
    dxd2d m_drv;
    using frame_buffer_type = bitmap<typename dxd2d::pixel_type>;
    using fb_color = color<typename frame_buffer_type::pixel_type>;
public:
    demo() :
            m_hwnd(nullptr),
            m_pDirect2dFactory(nullptr),
            m_pRenderTarget(nullptr),
            m_pLightSlateGrayBrush(nullptr),
            m_pCornflowerBlueBrush(nullptr),
            m_render_state(0),
            m_render_i(0) {
        m_frame_buffer = malloc(frame_buffer_type::sizeof_buffer({640,480}));
    }

    ~demo() {
        if(m_pLightSlateGrayBrush!=NULL) {
            m_pLightSlateGrayBrush->Release();
        }
        if(m_pCornflowerBlueBrush!=NULL) {
            m_pCornflowerBlueBrush->Release();
        }
        if(m_pRenderTarget!=NULL) {
            m_pRenderTarget->Release();
        }
        if(m_pDirect2dFactory!=NULL) {
            m_pDirect2dFactory->Release();
        }
        if(m_frame_buffer!=NULL) {
            free(m_frame_buffer);
        }
        
    }
    
   // Register the window class and call methods for instantiating drawing resources
    HRESULT Initialize() {
        HRESULT hr;
        
        // Initialize device-indpendent resources, such
        // as the Direct2D factory.
        hr = CreateDeviceIndependentResources();

        if (SUCCEEDED(hr))
        {
            // Register the window class.
            WNDCLASSEX wcex = { sizeof(WNDCLASSEX) };
            wcex.style         = CS_HREDRAW | CS_VREDRAW;
            wcex.lpfnWndProc   = demo::WndProc;
            wcex.cbClsExtra    = 0;
            wcex.cbWndExtra    = sizeof(LONG_PTR);
            wcex.hInstance     = HINST_THISCOMPONENT;
            wcex.hbrBackground = NULL;
            wcex.lpszMenuName  = NULL;
            wcex.hCursor       = LoadCursor(NULL, IDI_APPLICATION);
            wcex.lpszClassName = L"GFXDemo";

            RegisterClassEx(&wcex);


            // Because the CreateWindow function takes its size in pixels,
            // obtain the system DPI and use it to scale the window size.
            FLOAT dpiX, dpiY;

            // The factory returns the current system DPI. This is also the value it will use
            // to create its own windows.
            m_pDirect2dFactory->GetDesktopDpi(&dpiX, &dpiY);


            // Create the window.
            m_hwnd = CreateWindow(
                L"GFXDemo",
                L"GFX Demo",
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT,
                CW_USEDEFAULT,
                static_cast<UINT>(ceil(640.f * dpiX / 96.f)),
                static_cast<UINT>(ceil(480.f * dpiY / 96.f)),
                NULL,
                NULL,
                HINST_THISCOMPONENT,
                this
                );
            hr = m_hwnd ? S_OK : E_FAIL;
            

        }
        if (SUCCEEDED(hr))
        {
            ShowWindow(m_hwnd, SW_SHOWNORMAL);
            UpdateWindow(m_hwnd);
        }
        return hr;
    }
    // Process and dispatch messages
    void Run() {
        MSG msg;

        while (PeekMessage(&msg, NULL, 0, 0,PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);

            render_step();
            InvalidateRect(m_hwnd,NULL,FALSE) ;
        }
    }

private:
    
    // Initialize device-independent resources.
    HRESULT CreateDeviceIndependentResources() {
         HRESULT hr = S_OK;

        // Create a Direct2D factory.
        hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pDirect2dFactory);

        return hr;
    }

    // Initialize device-dependent resources.
    HRESULT CreateDeviceResources() {
        HRESULT hr = S_OK;

        if (m_pRenderTarget==NULL)
        {

            RECT rc;
            GetClientRect(m_hwnd, &rc);

            D2D1_SIZE_U size = D2D1::SizeU(
                rc.right - rc.left,
                rc.bottom - rc.top
                );

            // Create a Direct2D render target.
            hr = m_pDirect2dFactory->CreateHwndRenderTarget(
                D2D1::RenderTargetProperties(),
                D2D1::HwndRenderTargetProperties(m_hwnd, size),
                &m_pRenderTarget
                );

            if(SUCCEEDED(hr)) {
                ID2D1RenderTarget* prt;
                // linker choked on IID_ID2D1RenderTarget and I'm not sure what to link to, so this works instead
                static const GUID render_target_iid = {0x2cd90694, 0x12e2, 0x11dc, {0x9f, 0xed, 0x00, 0x11, 0x43, 0xa0, 0x55, 0xf9}};
                m_pRenderTarget->QueryInterface(render_target_iid,(void**)&prt);
                m_drv.render_target(prt);
            } else {
                m_drv.render_target(nullptr);
            }
        }
        return hr;
    }

    // Release device-dependent resource.
    void DiscardDeviceResources() {
       
        if(m_pRenderTarget!=NULL) {
            m_pRenderTarget->Release();
            m_pRenderTarget = NULL;
        }
    }

    // Draw content.
    HRESULT OnRender() {
        HRESULT hr = S_OK;
        hr = CreateDeviceResources();
        if (SUCCEEDED(hr))
        {
            draw::suspend(m_drv);
            frame_buffer_type fb({640,480},m_frame_buffer);
            draw::bitmap(m_drv,(srect16)m_drv.bounds(),fb,fb.bounds());
            if(draw::resume(m_drv)==gfx_result::canceled)
            {
                DiscardDeviceResources();
            }
        }    
        return hr;
    }

    // Resize the render target.
    void OnResize(
        UINT width,
        UINT height
        ) {
        if (m_pRenderTarget)
        {
            // Note: This method can fail, but it's okay to ignore the
            // error here, because the error will be returned again
            // the next time EndDraw is called.
            // m_pRenderTarget->Resize(D2D1::SizeU(width, height));
        }
    }

    // The windows procedure.
    static LRESULT CALLBACK WndProc(
        HWND hWnd,
        UINT message,
        WPARAM wParam,
        LPARAM lParam
        ) {

        LRESULT result = 0;

        if (message == WM_CREATE)
        {
            LPCREATESTRUCT pcs = (LPCREATESTRUCT)lParam;
            
            demo *pDemoApp = (demo *)pcs->lpCreateParams;

            ::SetWindowLongPtrW(
                hWnd,
                GWLP_USERDATA,
                reinterpret_cast<LONG_PTR>(pDemoApp)
                );
            
            result = 1;
        }
        else
        {
            bool wasHandled = false;
            
            demo *pDemoApp = reinterpret_cast<demo *>(static_cast<LONG_PTR>(
                ::GetWindowLongPtrW(
                    hWnd,
                    GWLP_USERDATA
                    )));

            
            if (pDemoApp)
            {
                switch (message)
                {
                case WM_SIZE:
                    {
                        UINT width = LOWORD(lParam);
                        UINT height = HIWORD(lParam);
                        pDemoApp->OnResize(width, height);
                    }
                    result = 0;
                    wasHandled = true;
                    break;
                
                case WM_DISPLAYCHANGE:
                    {
                        InvalidateRect(hWnd, NULL, FALSE);
                    }
                    result = 0;
                    wasHandled = true;
                    break;

                case WM_PAINT:
                    {
                        if(hWnd==pDemoApp->m_hwnd) {
                            pDemoApp->OnRender();
                            ValidateRect(hWnd, NULL);
                        }
                    }
                    result = 0;
                    wasHandled = true;
                    break;

                case WM_DESTROY:
                    {
                        PostQuitMessage(0);
                    }
                    result = 1;
                    wasHandled = true;
                    break;
                }
            }
            
            if (!wasHandled)
            {
                result = DefWindowProc(hWnd, message, wParam, lParam);
            }
        }

        return result;
    }
        
    // produced by request
    bool scroll_text_demo() {
        frame_buffer_type fb({640,480},m_frame_buffer);
            
        constexpr static const size16 bmp_size = {16,16};
        // you can use YbCbCr for example. It's lossy, so you'll want extra bits
        //using bmp_type = bitmap<ycbcr_pixel<HTCW_MAX_WORD>>;
        using bmp_type = frame_buffer_type;
        using bmp_color = color<typename bmp_type::pixel_type>;
        using bmpa_pixel_type = rgba_pixel<HTCW_MAX_WORD>;
        using bmpa_color = color<bmpa_pixel_type>;
        // declare the bitmap
        uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
        bmp_type bmp(bmp_size,bmp_buf);

        fb.clear(fb.bounds());
        // draw stuff
        bmp.clear(bmp.bounds()); // comment this out and check out the uninitialized RAM. It looks neat.
        bmpa_pixel_type col = bmpa_color::yellow;
        col.channelr<channel_name::A>(.5);
        // bounding info for the face
        srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1));
        rect16 ubounds(0,0,bounds.x2,bounds.y2);

        // draw the face
        draw::filled_ellipse(bmp,bounds,col);
        
        // draw the left eye
        srect16 eye_bounds_left(spoint16(bounds.width()/5,bounds.height()/5),ssize16(bounds.width()/5,bounds.height()/3));
        draw::filled_ellipse(bmp,eye_bounds_left,bmp_color::black);
        
        // draw the right eye
        srect16 eye_bounds_right(
            spoint16(
                bmp_size.width-eye_bounds_left.x1-eye_bounds_left.width(),
                eye_bounds_left.y1
            ),eye_bounds_left.dimensions());
        draw::filled_ellipse(bmp,eye_bounds_right,bmp_color::black);
        
        // draw the mouth
        srect16 mouth_bounds=bounds.inflate(-bounds.width()/7,-bounds.height()/8).normalize();
        // we need to clip part of the circle we'll be drawing
        srect16 mouth_clip(mouth_bounds.x1,mouth_bounds.y1+mouth_bounds.height()/(float)1.6,mouth_bounds.x2,mouth_bounds.y2);
        draw::ellipse(bmp,mouth_bounds,bmp_color::black,&mouth_clip);

        // do some alpha blended rectangles
        col = bmpa_color::red;
        col.channelr<channel_name::A>(.5);
        draw::filled_rectangle(bmp,srect16(spoint16(0,0),ssize16(bmp.dimensions().width,bmp.dimensions().height/4)),col);
        col = bmpa_color::blue;
        col.channelr<channel_name::A>(.5);
        draw::filled_rectangle(bmp,srect16(spoint16(0,0),ssize16(bmp.dimensions().width/4,bmp.dimensions().height)),col);
        col = bmpa_color::green;
        col.channelr<channel_name::A>(.5);
        draw::filled_rectangle(bmp,srect16(spoint16(0,bmp.dimensions().height-bmp.dimensions().height/4),ssize16(bmp.dimensions().width,bmp.dimensions().height/4)),col);
        col = bmpa_color::purple;
        col.channelr<channel_name::A>(.5);
        draw::filled_rectangle(bmp,srect16(spoint16(bmp.dimensions().width-bmp.dimensions().width/4,0),ssize16(bmp.dimensions().width/4,bmp.dimensions().height)),col);
        // uncomment to convert it to grayscale
        // resample<bmp_type,gsc_pixel<8>>(bmp);
        // uncomment to downsample
        // resample<bmp_type,rgb_pixel<8>>(bmp);
        srect16 new_bounds(0,0,63,63);

        // try using different values here. Bicubic yields the best visual result, but it's pretty slow. 
        // Bilinear is faster but better for shrinking images or changing sizes small amounts
        // Fast uses a nearest neighbor algorithm and is performant but looks choppy
        const bitmap_resize resize_type = 
            bitmap_resize::resize_bicubic;
            // bitmap_resize::resize_bilinear;
            //bitmap_resize::resize_fast;
        draw::bitmap(fb,new_bounds.center_horizontal((srect16)fb.bounds()).flip_vertical(),bmp,bmp.bounds(),resize_type);
        const gfx::font& f = Bm437_ATI_9x16_FON;
        const char* text = "copyright (C) 2021\r\nby honey the codewitch";
        ssize16 text_size = f.measure_text((ssize16)fb.dimensions(),text);
        srect16 text_rect = text_size.bounds().center((srect16)fb.bounds());
        int16_t text_start = text_rect.x1;
        
        // draw a polygon (a triangle in this case)
        // find the origin:
        const spoint16 porg = srect16(0,0,31,31)
                                .center_horizontal((srect16)fb.bounds())
                                    .offset(0,
                                        fb.dimensions().height-32)
                                            .top_left();
        // draw a 32x32 triangle by creating a path
        spoint16 path_points[] = {spoint16(0,31),spoint16(15,0),spoint16(31,31)};
        spath16 path(3,path_points);
        // offset it so it starts at the origin
        path.offset_inplace(porg.x,porg.y); 
        // draw it
        draw::filled_polygon(fb,path,fb_color::coral);

        bool first=true;
        while(true) {

        draw::filled_rectangle(fb,text_rect,fb_color::black);
            if(text_rect.x2>=320) {
            draw::filled_rectangle(fb,text_rect.offset(-320,0),fb_color::black);
            }

            text_rect=text_rect.offset(2,0);
            draw::text(fb,text_rect,text,f,fb_color::old_lace,fb_color::black,false);
            if(text_rect.x2>=320){
                draw::text(fb,text_rect.offset(-320,0),text,f,fb_color::old_lace,fb_color::black,false);
            }
            if(text_rect.x1>=320) {
                text_rect=text_rect.offset(-320,0);
                first=false;
            }
            
            if(!first && text_rect.x1>=text_start)
                break;
            
        }
        return true;
    }
    bool render_step() {
        frame_buffer_type fb({640,480},m_frame_buffer);
        switch(m_render_state) {
        case 0:
           m_render_i = 1;
           m_render_state = 1;
        case 1: {
                open_font f;
                // change this to your path
                file_stream fs(FONT_PATH);
                open_font::open(&fs,&f);
                draw::filled_rectangle(fb,(srect16)fb.bounds(),fb_color::white);
                const char* text = "DirectX GFX";
                float scale = f.scale(60);
                srect16 text_rect = f.measure_text((ssize16)fb.dimensions(),{0,0},
                                        text,scale).bounds();
                draw::text(fb,
                        text_rect.center((srect16)fb.bounds()),
                        {0,0},
                        text,
                        f,scale,
                        fb_color::dark_blue,fb_color::white,false);
                // free the font
                fs.close();
                m_render_state = 2;
            }
                break;
            case 2:
                if(m_render_i<100) {
                    // calculate our extents
                    srect16 r(m_render_i*(fb.dimensions().width/100.0),
                            m_render_i*(fb.dimensions().height/100.0),
                            fb.dimensions().width-m_render_i*(fb.dimensions().width/100.0)-1,
                            fb.dimensions().height-m_render_i*(fb.dimensions().height/100.0)-1);
                    // draw the four lines
                    draw::line(fb,srect16(0,r.y1,r.x1,fb.dimensions().height-1),fb_color::light_blue);
                    draw::line(fb,srect16(r.x2,0,fb.dimensions().width-1,r.y2),fb_color::hot_pink);
                    draw::line(fb,srect16(0,r.y2,r.x1,0),fb_color::pale_green);
                    draw::line(fb,srect16(fb.dimensions().width-1,r.y1,r.x2,fb.dimensions().height-1),fb_color::yellow);
                    m_render_i+=2;
                } else {
                    m_render_i = 0;
                    m_render_state = 0;
                    return false;
                }
        
        }
        return true;
    }
};
// Our application entry point.
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
  
 if (SUCCEEDED(CoInitialize(NULL)))
  {
      {
          demo app;

          if (SUCCEEDED(app.Initialize()))
          {
              app.Run();
          }
      }
      CoUninitialize();
      return 0;
  }
  return -1;
}
