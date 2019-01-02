package com.example.easha.tensorflowcluster;

import android.graphics.Bitmap;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.Surface;
import android.view.TextureView;
import android.widget.TextView;


public class MainActivity extends AppCompatActivity {
    private CameraDevice cameraDevice;
    private CameraManager cameraManager;
    private TextureView textureView;
    private String cameraID;

    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        textureView = (TextureView)findViewById(R.id.textureView);
        assert textureView != null;
        textureView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
                openCam();
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
                return false;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surface) {

            }
        });
    }

    private void createCameraPreview(){
        try{
            SurfaceTexture texture = this.textureView.getSurfaceTexture();
            assert texture != null;
            texture.setDefaultBufferSize(textureView.getHeight(), textureView.getWidth());
            Surface surface = new Surface(texture);


        }
        catch (){

        }
    }

    private void openCam() {

    }
}
