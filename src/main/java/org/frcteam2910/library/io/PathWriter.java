package org.frcteam2910.library.io;

import org.google.gson.Gson;
import org.google.gson.GsonBuilder;
import org.google.gson.JsonIOException;
import org.google.gson.JsonObject;
import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.library.control.Path;
import org.frcteam2910.library.control.PathSegment;
import org.frcteam2910.library.io.json.InterpolatingDoubleJsonHandler;
import org.frcteam2910.library.io.json.PathSegmentJsonHandler;
import org.frcteam2910.library.io.json.Rotation2JsonHandler;
import org.frcteam2910.library.io.json.SimpleMatrixJsonHandler;
import org.frcteam2910.library.math.Rotation2;
import org.frcteam2910.library.util.InterpolatingDouble;

import java.io.Flushable;
import java.io.IOException;
import java.io.Writer;

public final class PathWriter implements AutoCloseable, Flushable {
    private final Gson gson;
    private final Writer out;

    public PathWriter(Writer out) {
        this.gson = new GsonBuilder()
                .registerTypeAdapter(InterpolatingDouble.class, new InterpolatingDoubleJsonHandler())
                .registerTypeHierarchyAdapter(PathSegment.class, new PathSegmentJsonHandler())
                .registerTypeAdapter(Rotation2.class, new Rotation2JsonHandler())
                .registerTypeAdapter(SimpleMatrix.class, new SimpleMatrixJsonHandler())
                .enableComplexMapKeySerialization()
                .create();
        this.out = out;
    }

    public void write(Path path) throws IOException {
        try {
            JsonObject root = new JsonObject();
            root.add("segments", gson.toJsonTree(path.getSegments()));
            root.add("rotations", gson.toJsonTree(path.getRotationMap()));
            gson.toJson(root, out);
        } catch (JsonIOException e) {
            throw new IOException(e);
        }
    }

    @Override
    public void flush() throws IOException {
        out.flush();
    }

    @Override
    public void close() throws IOException {
        out.close();
    }
}
