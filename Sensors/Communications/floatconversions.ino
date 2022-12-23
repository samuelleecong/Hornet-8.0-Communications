float toFloat(uint8_t x) {
    return x / 255.0e7;
}

uint8_t fromFloat(float x) {
    if (x < 0) return 0;
    if (x > 1e7) return 255;
    return 255.0 * x; // this truncates; add 0.5 to round instead
}
