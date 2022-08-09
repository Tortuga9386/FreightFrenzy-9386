package org.firstinspires.ftc.teamcode.drive.opmodes;

import org.firstinspires.ftc.robotcore.external.Consumer;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Properties;
import java.util.Vector;

/**
 * Created by phurley on 11/10/17.
 */

public class ConstantsBase {

    public static String getFileName() {
        File file = new File("/sdcard", "calibration.properties");
        return file.getAbsolutePath();
    }

    private static String getResolvedFileName() {
        return getFileName().replaceFirst("^~", System.getProperty("user.home"));
    }

    public Field[] getFields() {
        return this.getClass().getDeclaredFields();
    }

    public List<Field> getStaticFields() {
        final List<Field> results = new Vector<>();

        withEachStaticField(new Consumer<Field>() {
            @Override
            public void accept(Field field) {
                results.add(field);
            }
        });

        return results;
    }

    private void withEachField(Consumer<Field> func) {
        for (Field field : this.getClass().getDeclaredFields()) {
            if (!java.lang.reflect.Modifier.isFinal(field.getModifiers()))
                func.accept(field);
        }
    }

    private void withEachStaticField(final Consumer<Field> func) {
        withEachField(new Consumer<Field>() {
            @Override
            public void accept(Field f) {
                if (java.lang.reflect.Modifier.isStatic(f.getModifiers())) {
                    func.accept(f);
                }
            }
        });
    }

    public void writeToFile() {
        final Properties data = new Properties();
        OutputStream output = null;

        try {
            (new File(getResolvedFileName())).delete();
            withEachStaticField(new Consumer<Field>() {
                @Override
                public void accept(Field f) {
                    String name = f.getName();
                    Object value = null;
                    try {
                        value = f.get(null);
                        data.setProperty(name, value.toString());
                    } catch (Exception e) {
                        System.err.println("Unable to get value for " + name);
                        e.printStackTrace();
                    }
                }
            });

            // save properties to project root folder
            output = new FileOutputStream(getResolvedFileName());
            Writer writer = new OutputStreamWriter(output);
            data.store(writer, "FTC Calibration Constants");

        } catch (IOException io) {
            io.printStackTrace();
        } finally {
            if (output != null) {
                try {
                    output.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public void setField(Field f, String value) {
        try {
            Class<?> klass = f.getType();
            if (value != null) {
                if (klass.equals(Integer.TYPE)) {
                    f.setInt(this, Integer.parseInt(value));
                } else if (klass.equals(Long.TYPE)) {
                    f.setLong(this, Long.parseLong(value));
                } else if (klass.equals(Double.TYPE)) {
                    f.setDouble(this, Double.parseDouble(value));
                } else if (klass.equals(Boolean.TYPE)) {
                    f.setBoolean(this, Boolean.parseBoolean(value));
                } else {
                    f.set(this, value);
                }
            }
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    protected void readFromFile() {
        InputStream input = null;

        try {
            input = new FileInputStream(getResolvedFileName());
            final Properties data = new Properties();
            data.load(input);

            withEachStaticField(new Consumer<Field>() {
                @Override
                public void accept(Field f) {
                    String name = f.getName();
                    setField(f, data.getProperty(name));
                }
            });

        } catch (FileNotFoundException err) {
            // (new ConstantsBase()).writeToFile();
            // do nothing here
        } catch (Exception err) {
            err.printStackTrace();
        } finally {
            if (input != null) {
                try {
                    input.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}