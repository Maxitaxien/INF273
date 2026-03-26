#include "verification/solution.h"
#include "datahandling/instance.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Point
{
    double x = 0.0;
    double y = 0.0;
};

struct Color
{
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

struct Bounds
{
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

double choose_orientation_angle(const std::vector<Point> &positions);

Point operator+(const Point &lhs, const Point &rhs)
{
    return Point{lhs.x + rhs.x, lhs.y + rhs.y};
}

Point operator-(const Point &lhs, const Point &rhs)
{
    return Point{lhs.x - rhs.x, lhs.y - rhs.y};
}

Point operator*(const Point &point, double scalar)
{
    return Point{point.x * scalar, point.y * scalar};
}

double norm(const Point &point)
{
    return std::sqrt(point.x * point.x + point.y * point.y);
}

Point normalize(const Point &point)
{
    const double length = norm(point);
    if (length <= 1e-12)
    {
        return Point{};
    }

    return Point{point.x / length, point.y / length};
}

Point rotate_point(const Point &point, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Point{
        point.x * c - point.y * s,
        point.x * s + point.y * c};
}

double cross(const Point &a, const Point &b, const Point &c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

class Canvas
{
public:
    Canvas(int width, int height, const Color &background)
        : width_(width), height_(height), pixels_(3 * width * height, 255)
    {
        for (int y = 0; y < height_; ++y)
        {
            for (int x = 0; x < width_; ++x)
            {
                set_pixel(x, y, background);
            }
        }
    }

    void fill_circle(double cx, double cy, double radius, const Color &color)
    {
        const int min_x = std::max(0, (int)(std::floor(cx - radius)));
        const int max_x = std::min(width_ - 1, (int)(std::ceil(cx + radius)));
        const int min_y = std::max(0, (int)(std::floor(cy - radius)));
        const int max_y = std::min(height_ - 1, (int)(std::ceil(cy + radius)));
        const double radius_sq = radius * radius;

        for (int y = min_y; y <= max_y; ++y)
        {
            for (int x = min_x; x <= max_x; ++x)
            {
                const double dx = x + 0.5 - cx;
                const double dy = y + 0.5 - cy;
                if (dx * dx + dy * dy <= radius_sq)
                {
                    set_pixel(x, y, color);
                }
            }
        }
    }

    void fill_square(double cx, double cy, double half_extent, const Color &color)
    {
        fill_rect(
            (int)(std::floor(cx - half_extent)),
            (int)(std::floor(cy - half_extent)),
            (int)(std::ceil(cx + half_extent)),
            (int)(std::ceil(cy + half_extent)),
            color);
    }

    void fill_triangle(const Point &a, const Point &b, const Point &c, const Color &color)
    {
        const int min_x = std::max(0, (int)(std::floor(std::min({a.x, b.x, c.x}))));
        const int max_x = std::min(width_ - 1, (int)(std::ceil(std::max({a.x, b.x, c.x}))));
        const int min_y = std::max(0, (int)(std::floor(std::min({a.y, b.y, c.y}))));
        const int max_y = std::min(height_ - 1, (int)(std::ceil(std::max({a.y, b.y, c.y}))));

        for (int y = min_y; y <= max_y; ++y)
        {
            for (int x = min_x; x <= max_x; ++x)
            {
                const Point p{x + 0.5, y + 0.5};
                const double c1 = cross(a, b, p);
                const double c2 = cross(b, c, p);
                const double c3 = cross(c, a, p);
                const bool has_neg = c1 < 0 || c2 < 0 || c3 < 0;
                const bool has_pos = c1 > 0 || c2 > 0 || c3 > 0;
                if (!(has_neg && has_pos))
                {
                    set_pixel(x, y, color);
                }
            }
        }
    }

    void draw_segment(const Point &start, const Point &end, const Color &color, double thickness)
    {
        const double length = norm(end - start);
        const int steps = std::max(1, (int)(std::ceil(length * 2.0)));
        const double radius = std::max(1.0, thickness / 2.0);

        for (int i = 0; i <= steps; ++i)
        {
            const double t = (double)(i) / steps;
            const Point point{
                start.x + (end.x - start.x) * t,
                start.y + (end.y - start.y) * t};
            fill_circle(point.x, point.y, radius, color);
        }
    }

    void draw_arrow(
        const Point &start,
        const Point &end,
        const Color &color,
        double thickness,
        double head_length,
        double head_width)
    {
        const Point direction = normalize(end - start);
        if (norm(direction) <= 1e-12)
        {
            return;
        }

        Point shaft_end = end - direction * head_length;
        if (norm(shaft_end - start) <= 1.0)
        {
            shaft_end = start;
        }

        draw_segment(start, shaft_end, color, thickness);

        const Point perpendicular{-direction.y, direction.x};
        const Point left = shaft_end + perpendicular * (head_width / 2.0);
        const Point right = shaft_end - perpendicular * (head_width / 2.0);
        fill_triangle(end, left, right, color);
    }

    void draw_dotted_arrow(
        const Point &start,
        const Point &end,
        const Color &color,
        double thickness,
        double dash_length,
        double gap_length,
        double head_length,
        double head_width)
    {
        const Point direction = normalize(end - start);
        const double total_length = norm(end - start);
        if (total_length <= 1e-12)
        {
            return;
        }

        const double line_length = std::max(0.0, total_length - head_length);
        double travelled = 0.0;
        while (travelled < line_length)
        {
            const double dash_end = std::min(line_length, travelled + dash_length);
            const Point dash_start = start + direction * travelled;
            const Point dash_finish = start + direction * dash_end;
            draw_segment(dash_start, dash_finish, color, thickness);
            travelled = dash_end + gap_length;
        }

        const Point base = end - direction * head_length;
        const Point perpendicular{-direction.y, direction.x};
        const Point left = base + perpendicular * (head_width / 2.0);
        const Point right = base - perpendicular * (head_width / 2.0);
        fill_triangle(end, left, right, color);
    }

    void draw_text_centered(double cx, double cy, const std::string &text, int scale, const Color &color)
    {
        if (text.empty())
        {
            return;
        }

        const int gap = scale;
        const int glyph_width = 3 * scale;
        const int glyph_height = 5 * scale;
        const int total_width = (int)(text.size()) * glyph_width + ((int)(text.size()) - 1) * gap;
        const int start_x = (int)(std::round(cx - total_width / 2.0));
        const int start_y = (int)(std::round(cy - glyph_height / 2.0));

        for (int i = 0; i < (int)(text.size()); ++i)
        {
            const char ch = text[i];
            if (ch < '0' || ch > '9')
            {
                continue;
            }

            const auto &bitmap = digit_bitmaps()[ch - '0'];
            const int glyph_x = start_x + i * (glyph_width + gap);

            for (int row = 0; row < 5; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    if (bitmap[row][col] == '1')
                    {
                        fill_rect(
                            glyph_x + col * scale,
                            start_y + row * scale,
                            glyph_x + (col + 1) * scale - 1,
                            start_y + (row + 1) * scale - 1,
                            color);
                    }
                }
            }
        }
    }

    bool save_jpg(const std::string &path, int quality) const
    {
        return stbi_write_jpg(path.c_str(), width_, height_, 3, pixels_.data(), quality) != 0;
    }

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<std::uint8_t> pixels_;

    static const std::array<std::array<std::string, 5>, 10> &digit_bitmaps()
    {
        static const std::array<std::array<std::string, 5>, 10> bitmaps = {{
            {{"111", "101", "101", "101", "111"}},
            {{"010", "110", "010", "010", "111"}},
            {{"111", "001", "111", "100", "111"}},
            {{"111", "001", "111", "001", "111"}},
            {{"101", "101", "111", "001", "001"}},
            {{"111", "100", "111", "001", "111"}},
            {{"111", "100", "111", "101", "111"}},
            {{"111", "001", "001", "001", "001"}},
            {{"111", "101", "111", "101", "111"}},
            {{"111", "101", "111", "001", "111"}},
        }};
        return bitmaps;
    }

    void set_pixel(int x, int y, const Color &color)
    {
        if (x < 0 || x >= width_ || y < 0 || y >= height_)
        {
            return;
        }

        const int idx = 3 * (y * width_ + x);
        pixels_[idx] = color.r;
        pixels_[idx + 1] = color.g;
        pixels_[idx + 2] = color.b;
    }

    void fill_rect(int min_x, int min_y, int max_x, int max_y, const Color &color)
    {
        const int clamped_min_x = std::max(0, min_x);
        const int clamped_max_x = std::min(width_ - 1, max_x);
        const int clamped_min_y = std::max(0, min_y);
        const int clamped_max_y = std::min(height_ - 1, max_y);

        for (int y = clamped_min_y; y <= clamped_max_y; ++y)
        {
            for (int x = clamped_min_x; x <= clamped_max_x; ++x)
            {
                set_pixel(x, y, color);
            }
        }
    }
};

std::vector<std::vector<double>> build_gram_matrix(const Instance &instance)
{
    const int size = instance.n + 1;
    std::vector<std::vector<double>> squared(size, std::vector<double>(size, 0.0));
    std::vector<double> row_mean(size, 0.0);
    double total_mean = 0.0;

    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            const double value = (double)(instance.truck_matrix[i][j]);
            squared[i][j] = value * value;
            row_mean[i] += squared[i][j];
            total_mean += squared[i][j];
        }
        row_mean[i] /= size;
    }
    total_mean /= (double)(size * size);

    std::vector<std::vector<double>> gram(size, std::vector<double>(size, 0.0));
    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            gram[i][j] = -0.5 * (squared[i][j] - row_mean[i] - row_mean[j] + total_mean);
        }
    }

    return gram;
}

void jacobi_eigendecomposition(
    std::vector<std::vector<double>> matrix,
    std::vector<double> &eigenvalues,
    std::vector<std::vector<double>> &eigenvectors)
{
    const int size = (int)(matrix.size());
    eigenvectors.assign(size, std::vector<double>(size, 0.0));
    for (int i = 0; i < size; ++i)
    {
        eigenvectors[i][i] = 1.0;
    }

    const int max_iterations = std::max(100, 50 * size * size);
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        double best = 0.0;
        int p = 0;
        int q = 1;

        for (int i = 0; i < size; ++i)
        {
            for (int j = i + 1; j < size; ++j)
            {
                const double candidate = std::abs(matrix[i][j]);
                if (candidate > best)
                {
                    best = candidate;
                    p = i;
                    q = j;
                }
            }
        }

        if (best <= 1e-9)
        {
            break;
        }

        const double app = matrix[p][p];
        const double aqq = matrix[q][q];
        const double apq = matrix[p][q];
        const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(angle);
        const double s = std::sin(angle);

        for (int k = 0; k < size; ++k)
        {
            if (k == p || k == q)
            {
                continue;
            }

            const double akp = matrix[k][p];
            const double akq = matrix[k][q];
            matrix[k][p] = c * akp - s * akq;
            matrix[p][k] = matrix[k][p];
            matrix[k][q] = s * akp + c * akq;
            matrix[q][k] = matrix[k][q];
        }

        matrix[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        matrix[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        matrix[p][q] = 0.0;
        matrix[q][p] = 0.0;

        for (int k = 0; k < size; ++k)
        {
            const double vip = eigenvectors[k][p];
            const double viq = eigenvectors[k][q];
            eigenvectors[k][p] = c * vip - s * viq;
            eigenvectors[k][q] = s * vip + c * viq;
        }
    }

    eigenvalues.assign(size, 0.0);
    for (int i = 0; i < size; ++i)
    {
        eigenvalues[i] = matrix[i][i];
    }
}

std::vector<Point> fallback_positions(const Instance &instance)
{
    std::vector<Point> positions(instance.n + 1);
    double y = 0.0;
    for (int node = 0; node <= instance.n; ++node)
    {
        positions[node] = Point{0.0, y};
        if (node < instance.n)
        {
            y += std::max(1.0, (double)(instance.truck_matrix[node][node + 1]));
        }
    }
    return positions;
}

std::vector<Point> infer_positions(const Instance &instance)
{
    const int size = instance.n + 1;
    if ((int)(instance.truck_matrix.size()) != size)
    {
        return {};
    }

    for (const auto &row : instance.truck_matrix)
    {
        if ((int)(row.size()) != size)
        {
            return {};
        }
    }

    if (size == 1)
    {
        return std::vector<Point>(1, Point{0.0, 0.0});
    }

    const std::vector<std::vector<double>> gram = build_gram_matrix(instance);
    std::vector<double> eigenvalues;
    std::vector<std::vector<double>> eigenvectors;
    jacobi_eigendecomposition(gram, eigenvalues, eigenvectors);

    std::vector<int> order(size);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        return eigenvalues[lhs] > eigenvalues[rhs];
    });

    if (eigenvalues[order[0]] <= 1e-9)
    {
        return fallback_positions(instance);
    }

    std::vector<Point> positions(size, Point{});
    const double lambda_x = std::sqrt(std::max(0.0, eigenvalues[order[0]]));
    const double lambda_y = order.size() > 1
        ? std::sqrt(std::max(0.0, eigenvalues[order[1]]))
        : 0.0;

    for (int node = 0; node < size; ++node)
    {
        positions[node].x = eigenvectors[node][order[0]] * lambda_x;
        positions[node].y = (order.size() > 1 ? eigenvectors[node][order[1]] * lambda_y : 0.0);
    }

    const Point depot = positions[0];
    for (Point &point : positions)
    {
        point = point - depot;
    }

    const double best_angle = choose_orientation_angle(positions);

    for (Point &point : positions)
    {
        point = rotate_point(point, best_angle);
    }

    return positions;
}

Bounds compute_bounds(const std::vector<Point> &points)
{
    Bounds bounds{};
    if (points.empty())
    {
        return bounds;
    }

    bounds.min_x = bounds.max_x = points[0].x;
    bounds.min_y = bounds.max_y = points[0].y;

    for (const Point &point : points)
    {
        bounds.min_x = std::min(bounds.min_x, point.x);
        bounds.max_x = std::max(bounds.max_x, point.x);
        bounds.min_y = std::min(bounds.min_y, point.y);
        bounds.max_y = std::max(bounds.max_y, point.y);
    }

    return bounds;
}

double choose_orientation_angle(const std::vector<Point> &positions)
{
    if (positions.size() <= 1)
    {
        return 0.0;
    }

    const int customer_count = (int)(positions.size()) - 1;
    const int steps = 2880;
    int best_positive_count = -1;
    double best_min_y = -std::numeric_limits<double>::infinity();
    double best_mean_y = -std::numeric_limits<double>::infinity();
    double best_angle = 0.0;

    for (int step = 0; step < steps; ++step)
    {
        const double angle = 2.0 * kPi * step / steps;
        int positive_count = 0;
        double min_y = std::numeric_limits<double>::infinity();
        double mean_y = 0.0;

        for (int node = 1; node < (int)(positions.size()); ++node)
        {
            const Point rotated = rotate_point(positions[node], angle);
            if (rotated.y >= -1e-9)
            {
                ++positive_count;
            }
            min_y = std::min(min_y, rotated.y);
            mean_y += rotated.y;
        }

        mean_y /= std::max(1, customer_count);

        bool better = false;
        if (positive_count > best_positive_count)
        {
            better = true;
        }
        else if (positive_count == best_positive_count)
        {
            const bool all_customers_above_depot = positive_count == customer_count;
            const bool best_has_all_customers = best_positive_count == customer_count;

            if (all_customers_above_depot && best_has_all_customers)
            {
                // When every customer can stay above the depot, keep the lowest one close to it.
                better = min_y < best_min_y - 1e-9 ||
                    (std::abs(min_y - best_min_y) <= 1e-9 && mean_y > best_mean_y + 1e-9);
            }
            else
            {
                // Otherwise minimize how far any customer falls below the depot.
                better = min_y > best_min_y + 1e-9 ||
                    (std::abs(min_y - best_min_y) <= 1e-9 && mean_y > best_mean_y + 1e-9);
            }
        }

        if (better)
        {
            best_positive_count = positive_count;
            best_min_y = min_y;
            best_mean_y = mean_y;
            best_angle = angle;
        }
    }

    return best_angle;
}

Point to_screen(
    const Point &world,
    const Bounds &bounds,
    int width,
    int height,
    double margin,
    double scale)
{
    const double x = margin + (world.x - bounds.min_x) * scale;
    const double y = height - margin - (world.y - bounds.min_y) * scale;
    return Point{x, y};
}
}

bool Solution::save_visualization(const Instance &instance, const std::string &output_path) const
{
    if (truck_route.empty())
    {
        return false;
    }

    std::vector<Point> positions = infer_positions(instance);
    if (positions.size() != (size_t)(instance.n + 1))
    {
        return false;
    }

    const int width = std::max(1800, 1400 + 12 * instance.n);
    const int height = std::max(1100, 950 + 8 * instance.n);
    const double customer_radius = std::clamp(36.0 - 2.4 * std::sqrt((double)(instance.n)), 10.0, 30.0);
    const double depot_half_extent = customer_radius + 4.0;
    const double truck_thickness = std::clamp(customer_radius * 0.28, 3.0, 8.0);
    const double truck_head_length = std::clamp(customer_radius * 0.95, 12.0, 26.0);
    const double truck_head_width = std::clamp(customer_radius * 0.75, 10.0, 20.0);
    const double drone_thickness = std::clamp(customer_radius * 0.14, 2.0, 4.0);
    const double drone_dash_length = std::clamp(customer_radius * 0.70, 8.0, 20.0);
    const double drone_gap_length = std::clamp(customer_radius * 0.45, 5.0, 12.0);
    const double drone_head_length = std::clamp(customer_radius * 0.70, 10.0, 18.0);
    const double drone_head_width = std::clamp(customer_radius * 0.55, 8.0, 14.0);
    const double margin = std::max(70.0, customer_radius * 2.5);
    const Bounds bounds = compute_bounds(positions);
    const double x_span = std::max(1.0, bounds.max_x - bounds.min_x);
    const double y_span = std::max(1.0, bounds.max_y - bounds.min_y);
    const double scale = std::min(
        (width - 2.0 * margin) / x_span,
        (height - 2.0 * margin) / y_span);

    std::vector<Point> screen_positions(positions.size());
    for (int node = 0; node <= instance.n; ++node)
    {
        screen_positions[node] = to_screen(positions[node], bounds, width, height, margin, scale);
    }

    const Color white{255, 255, 255};
    const Color black{0, 0, 0};
    const Color blue{52, 120, 246};
    const std::array<Color, 4> drone_colors = {black, blue, Color{0, 140, 90}, Color{160, 70, 0}};
    Canvas canvas(width, height, white);

    for (int drone = 0; drone < (int)(drones.size()); ++drone)
    {
        const DroneCollection &collection = drones[drone];
        const int flight_count = std::min({
            (int)(collection.launch_indices.size()),
            (int)(collection.deliver_nodes.size()),
            (int)(collection.land_indices.size())});
        const Color &drone_color = drone_colors[std::min(drone, (int)(drone_colors.size()) - 1)];

        for (int flight = 0; flight < flight_count; ++flight)
        {
            const int launch_idx = collection.launch_indices[flight];
            const int land_idx = collection.land_indices[flight];
            const int deliver = collection.deliver_nodes[flight];

            if (launch_idx < 0 || land_idx < 0 ||
                launch_idx >= (int)(truck_route.size()) || land_idx >= (int)(truck_route.size()) ||
                deliver < 0 || deliver > instance.n)
            {
                continue;
            }

            const int launch_node = truck_route[launch_idx];
            const int land_node = truck_route[land_idx];
            canvas.draw_dotted_arrow(
                screen_positions[launch_node],
                screen_positions[deliver],
                drone_color,
                drone_thickness,
                drone_dash_length,
                drone_gap_length,
                drone_head_length,
                drone_head_width);
            canvas.draw_dotted_arrow(
                screen_positions[deliver],
                screen_positions[land_node],
                drone_color,
                drone_thickness,
                drone_dash_length,
                drone_gap_length,
                drone_head_length,
                drone_head_width);
        }
    }

    if (truck_route.size() >= 2)
    {
        for (int idx = 0; idx < (int)(truck_route.size()); ++idx)
        {
            const int from = truck_route[idx];
            const int to = (idx + 1 < (int)(truck_route.size()))
                ? truck_route[idx + 1]
                : 0;

            if (from < 0 || from > instance.n || to < 0 || to > instance.n)
            {
                continue;
            }

            canvas.draw_arrow(
                screen_positions[from],
                screen_positions[to],
                black,
                truck_thickness,
                truck_head_length,
                truck_head_width);
        }
    }

    for (int node = 1; node <= instance.n; ++node)
    {
        canvas.fill_circle(screen_positions[node].x, screen_positions[node].y, customer_radius, black);
    }
    canvas.fill_square(screen_positions[0].x, screen_positions[0].y, depot_half_extent, black);

    for (int node = 0; node <= instance.n; ++node)
    {
        const int label_scale = std::max(2, (int)(std::round(customer_radius / 6.0)));
        canvas.draw_text_centered(
            screen_positions[node].x,
            screen_positions[node].y,
            std::to_string(node),
            label_scale,
            white);
    }

    const std::filesystem::path output(output_path);
    if (output.has_parent_path())
    {
        std::filesystem::create_directories(output.parent_path());
    }

    return canvas.save_jpg(output_path, 95);
}
