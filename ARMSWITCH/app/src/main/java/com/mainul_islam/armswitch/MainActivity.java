package com.mainul_islam.armswitch;

import androidx.activity.ComponentActivity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends ComponentActivity {

    // ====== CONFIGURE THESE ======
    private static final String CORRECT_PASSWORD = "1234";
    private volatile int        armHoldMs        = 500;  // settable 100–1000 ms via GUI
    private static final int    CONNECT_TIMEOUT  = 3000;
    private static final int    READ_TIMEOUT     = 5000;
    private static final int    SLIDE_THRESHOLD  = 95;   // 0–100
    // =============================

    private TextView passwordDisplay, statusDisplay;
    private EditText ipAddressInput, portInput;
    private SeekBar  armSlider;
    private SeekBar  armTimeSlider;
    private TextView armTimeLabel;
    private Button   btnFire, btnReset;

    private final StringBuilder enteredPassword = new StringBuilder();

    private boolean passwordVerified  = false;
    private boolean armed             = false;   // slider has been pulled past threshold
    private volatile boolean isArming = false;   // TCP connection in flight
    private boolean lockedOut         = false;   // after a failed connection

    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final Handler ui = new Handler(Looper.getMainLooper());

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        passwordDisplay = findViewById(R.id.passwordDisplay);
        statusDisplay   = findViewById(R.id.statusDisplay);
        ipAddressInput  = findViewById(R.id.ipAddressInput);
        portInput       = findViewById(R.id.portInput);
        armSlider       = findViewById(R.id.armSlider);
        armTimeSlider   = findViewById(R.id.armTimeSlider);
        armTimeLabel    = findViewById(R.id.armTimeLabel);
        btnFire         = findViewById(R.id.btnFire);
        btnReset        = findViewById(R.id.btnReset);

        int[] numIds = {R.id.btn0, R.id.btn1, R.id.btn2, R.id.btn3, R.id.btn4,
                R.id.btn5, R.id.btn6, R.id.btn7, R.id.btn8, R.id.btn9};
        for (int i = 0; i < 10; i++) {
            final int digit = i;
            findViewById(numIds[i]).setOnClickListener(v -> onDigitPressed(digit));
        }
        findViewById(R.id.btnClear).setOnClickListener(v -> clearPassword());
        findViewById(R.id.btnEnter).setOnClickListener(v -> verifyPassword());

        btnFire.setOnClickListener(v -> fire());
        btnReset.setOnClickListener(v -> reset());

        armSlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override public void onProgressChanged(SeekBar bar, int p, boolean fromUser) {}
            @Override public void onStartTrackingTouch(SeekBar bar) {}
            @Override public void onStopTrackingTouch(SeekBar bar) {
                if (armed || !passwordVerified || isArming || lockedOut) return;
                if (bar.getProgress() >= SLIDE_THRESHOLD) {
                    bar.setProgress(100);
                    armed = true;
                    statusDisplay.setText("ARMED — press FIRE");
                    updateUI();
                } else {
                    bar.setProgress(0);
                }
            }
        });

        armTimeSlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override public void onProgressChanged(SeekBar bar, int p, boolean fromUser) {
                armHoldMs = 10 + p;   // 0–4990 -> 10–5000 ms
                armTimeLabel.setText("ARM TIME: " + armHoldMs + " ms");
            }
            @Override public void onStartTrackingTouch(SeekBar bar) {}
            @Override public void onStopTrackingTouch(SeekBar bar) {}
        });
        armHoldMs = 10 + armTimeSlider.getProgress();
        armTimeLabel.setText("ARM TIME: " + armHoldMs + " ms");

        updateUI();
    }

    // ---------- keypad / password ----------

    private void onDigitPressed(int digit) {
        if (passwordVerified || isArming || lockedOut) return;
        if (enteredPassword.length() < 12) {
            enteredPassword.append(digit);
            passwordDisplay.setText(mask(enteredPassword.length()));
        }
    }

    private String mask(int n) {
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < n; i++) s.append('*');
        return s.toString();
    }

    private void clearPassword() {
        if (passwordVerified || isArming || lockedOut) return;
        enteredPassword.setLength(0);
        passwordDisplay.setText("");
    }

    private void verifyPassword() {
        if (passwordVerified || isArming || lockedOut) return;
        if (enteredPassword.toString().equals(CORRECT_PASSWORD)) {
            passwordVerified = true;
            statusDisplay.setText("Password OK! READY FOR ARM");
            updateUI();
        } else {
            Toast.makeText(this, "Wrong password", Toast.LENGTH_SHORT).show();
            enteredPassword.setLength(0);
            passwordDisplay.setText("");
        }
    }

    // ---------- fire / reset ----------

    private void fire() {
        if (!armed || isArming || lockedOut) return;
        final String ip = ipAddressInput.getText().toString().trim();
        final String portStr = portInput.getText().toString().trim();
        if (ip.isEmpty() || portStr.isEmpty()) {
            Toast.makeText(this, "Set IP and port", Toast.LENGTH_SHORT).show();
            return;
        }
        final int port;
        try { port = Integer.parseInt(portStr); }
        catch (NumberFormatException e) {
            Toast.makeText(this, "Bad port", Toast.LENGTH_SHORT).show();
            return;
        }
        isArming = true;
        statusDisplay.setText("Connecting…");
        updateUI();
        executor.submit(() -> sendFireTcp(ip, port));
    }

    private void reset() {
        if (isArming) return;
        passwordVerified = false;
        armed = false;
        lockedOut = false;
        enteredPassword.setLength(0);
        passwordDisplay.setText("");
        statusDisplay.setText("IDLE — enter password");
        armSlider.setProgress(0);
        updateUI();
    }

    private void updateUI() {
        boolean canSlide = passwordVerified && !armed && !isArming && !lockedOut;
        boolean canFire  = armed && !isArming && !lockedOut;
        armSlider.setEnabled(canSlide);
        btnFire.setEnabled(canFire);
        btnReset.setEnabled(!isArming);
    }

    // ---------- TCP ----------

    private void sendFireTcp(String ip, int port) {
        Socket socket = null;
        try {
            socket = new Socket();
            socket.connect(new InetSocketAddress(ip, port), CONNECT_TIMEOUT);
            socket.setSoTimeout(READ_TIMEOUT);

            OutputStream out = socket.getOutputStream();
            BufferedReader in = new BufferedReader(
                    new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));

            final int holdMs = armHoldMs;
            out.write(("FIRE " + holdMs + "\n").getBytes(StandardCharsets.UTF_8));
            out.flush();
            ui.post(() -> statusDisplay.setText("Sent FIRE, waiting for ack…"));

            String reply = in.readLine();
            if (reply == null) throw new IOException("Pi closed connection");
            reply = reply.trim();
            if (!reply.equals("OK")) throw new IOException("Pi replied: " + reply);

            ui.post(() -> statusDisplay.setText("FIRING — holding " + holdMs + "ms…"));
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < holdMs) {
                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
            }

            ui.post(() -> {
                statusDisplay.setText("DONE");
                isArming = false;
                reset();
            });
        } catch (Exception e) {
            final String reason = e.getClass().getSimpleName() + ": " + e.getMessage();
            ui.post(() -> {
                statusDisplay.setText("FAIL — " + reason);
                isArming = false;
                lockedOut = true;
                updateUI();
            });
        } finally {
            if (socket != null) try { socket.close(); } catch (IOException ignored) {}
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        executor.shutdownNow();
    }
}