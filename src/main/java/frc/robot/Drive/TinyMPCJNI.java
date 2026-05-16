package frc.robot.Drive;

import edu.wpi.first.util.RuntimeLoader;

public class TinyMPCJNI {
    private static final String LIBRARY_NAME = "tinympc_jni";
    private static boolean loaded = false;
    private static Throwable loadError = null;

    static {
        try {
            RuntimeLoader.loadLibrary(LIBRARY_NAME);
            loaded = true;
        } catch (Throwable t) {
            loadError = t;
            loaded = false;
        }
    }

    // Native method declarations
    private static native void initializeNative();
    private static native double[] solveNative(double[] x0, double[] xref_flat);

    public static boolean isAvailable() {
        return loaded;
    }

    public static Throwable getLoadError() {
        return loadError;
    }

    public static void initialize() {
        if (!loaded) {
            throw new IllegalStateException("TinyMPC JNI library failed to load", loadError);
        }
        initializeNative();
    }

    public static double[] getControl(double[] x0, double[][] xref) {
        if (!loaded) {
            throw new IllegalStateException("TinyMPC JNI library failed to load", loadError);
        }
        int horizon = 30;
        int states = 6;
        double[] flat = new double[horizon * states];

        // Column-Major Packing
        // We iterate through each state (row), then all its time steps (columns)
        int index = 0;
        for (int j = 0; j < horizon; j++) {
            for (int i = 0; i < states; i++) {
                flat[index++] = xref[j][i];
            }
        }
        //System.out.println(Arrays.toString(xref[0]));
        return solveNative(x0, flat);
    }
}