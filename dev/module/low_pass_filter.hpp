//
// Created by liuzikai on 7/14/21.
//

#ifndef META_INFANTRY_LOW_PASS_FILTER_HPP
#define META_INFANTRY_LOW_PASS_FILTER_HPP

class LowPassFilteredValue {
public:
    LowPassFilteredValue() = default;

    explicit LowPassFilteredValue(float alpha) : alpha_(alpha) {}

    /**
     * Set alpha value.
     * @param alpha  Weight for the last data.
     */
    void set_alpha(float alpha) { alpha_ = alpha; }

    void update(float val) {
        val_ = valValid_ ? (val_ * alpha_ + val * (1 - alpha_)) : val;
        valValid_ = true;
    }

    float get() const { return val_; }

    void direct_set(float val) {
        val_ = val;
        valValid_ = true;
    }

    void reset() {
        val_ = 0;
        valValid_ = false;
    }

private:
    float alpha_ = 0;
    bool valValid_ = false;
    float val_ = 0;
};

#endif //META_INFANTRY_LOW_PASS_FILTER_HPP
