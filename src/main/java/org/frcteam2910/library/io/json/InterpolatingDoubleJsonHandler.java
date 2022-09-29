package org.frcteam2910.library.io.json;

import org.frcteam2910.library.util.InterpolatingDouble;
import org.google.gson.*;

import java.lang.reflect.Type;

public final class InterpolatingDoubleJsonHandler implements JsonDeserializer<InterpolatingDouble>, JsonSerializer<InterpolatingDouble> {
    @Override
    public JsonElement serialize(InterpolatingDouble src, Type typeOfSrc, JsonSerializationContext context) {
        return new JsonPrimitive(src.value);
    }

    @Override
    public InterpolatingDouble deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        return new InterpolatingDouble(json.getAsDouble());
    }
}
