#include "tests/test_framework.h"
#include "logging/csv_logger.h"
#include <string.h>

TEST(test_open_writes_header_row) {
    csv_logger_t log;
    CHECK(csv_logger_open(&log, "test_output_header.csv"));
    csv_logger_close(&log);
    FILE *f = fopen("test_output_header.csv", "r");
    CHECK(f != NULL);
    char line[512];
    fgets(line, sizeof(line), f);
    CHECK(strstr(line, "t_s") != NULL);
    CHECK(strstr(line, "raw_is_stale") != NULL);
    CHECK(strstr(line, "servo_angle_deg") != NULL);
    fclose(f);
}

TEST(test_write_row_produces_expected_column_count) {
    csv_logger_t log;
    csv_logger_open(&log, "test_output_row.csv");
    csv_logger_write_row(&log, 0.01, 1.0, 0.1, 0.1, false, 0.1, 0.5, -0.4, -0.6, -0.5, 60.0);
    csv_logger_close(&log);
    FILE *f = fopen("test_output_row.csv", "r");
    char header[512], row[512];
    fgets(header, sizeof(header), f);
    fgets(row, sizeof(row), f);
    int commas = 0;
    for (char *p = row; *p; p++) if (*p == ',') commas++;
    CHECK(commas == 10); /* 11 columns -> 10 commas */
    fclose(f);
}

TEST(test_open_stream_writes_hw_header_row) {
    csv_logger_t log;
    FILE *f = tmpfile();
    CHECK(f != NULL);
    CHECK(csv_logger_open_stream(&log, f));
    fflush(log.fp);
    rewind(f);
    char line[512];
    fgets(line, sizeof(line), f);
    CHECK(strstr(line, "measured_range_m") != NULL);
    CHECK(strstr(line, "true_range_m") == NULL);
    fclose(f);
}

TEST(test_write_row_hw_produces_expected_column_count) {
    csv_logger_t log;
    FILE *f = tmpfile();
    CHECK(f != NULL);
    CHECK(csv_logger_open_stream(&log, f));
    csv_logger_write_row_hw(&log, 0.01, 0.9, 0.1, false, 0.1, 0.5, -0.4, -0.6, -0.5, 60.0);
    fflush(log.fp);
    rewind(f);
    char header[512], row[512];
    fgets(header, sizeof(header), f);
    fgets(row, sizeof(row), f);
    int commas = 0;
    for (char *p = row; *p; p++) if (*p == ',') commas++;
    CHECK(commas == 9); /* 10 columns -> 9 commas */
    fclose(f);
}

int main(void) {
    RUN_TEST(test_open_writes_header_row);
    RUN_TEST(test_write_row_produces_expected_column_count);
    RUN_TEST(test_open_stream_writes_hw_header_row);
    RUN_TEST(test_write_row_hw_produces_expected_column_count);
    TEST_SUMMARY();
    return 0;
}
