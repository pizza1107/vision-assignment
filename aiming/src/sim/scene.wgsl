@group(0) @binding(0) var artwork: texture_2d<f32>;
@group(0) @binding(1) var artwork_sampler: sampler;
struct Varying {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) uv: vec2<f32>,
};
@vertex fn vs(@location(0) position: vec4<f32>, @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>) -> Varying {
    return Varying(position, color, uv);
}
@fragment fn fs(input: Varying) -> @location(0) vec4<f32> {
    if input.uv.x >= 0.0 {
        let ink = textureSample(artwork, artwork_sampler, input.uv);
        if ink.a < 0.5 { discard; }
        return vec4<f32>(input.color.rgb * ink.rgb, 1.0);
    }
    return input.color;
}
