library(tidyverse)
library(stringr)
library(foreach)

files <- foreach(filename = list.files("data", "*agents,d=*", full.names = TRUE), .combine = c) %do% {
  list(list(filename = filename,
            seed = filename %>% str_replace(".*s=(\\d+).*", "\\1") %>% as.numeric,
            lambda = filename %>% str_replace(".*l=(\\d+).*", "\\1") %>% as.numeric,
            dimensions = filename %>%
              str_replace(".*d=(\\d+),(\\d+),(\\d+),(\\d+).*", "\\1,\\2,\\3,\\4") %>%
              str_split(",") %>%
              unlist() %>%
              as.numeric
            ))
}

data <- foreach(file = files, .combine = rbind) %do% {
  with(file, {
    read_csv(filename, col_types = cols(.default = col_integer())) %>%
      mutate(seed = seed, lambda = lambda,
             X = dimensions[1], Y = dimensions[2], Z = dimensions[3],
             time_window = dimensions[4])
  })
}

write_csv(data, "data/agents,summary.csv.gz")

# data %>%
#   group_by(lambda, t) %>%
#   summarize(avg = mean(count), .groups = "drop") %>% {
#     ggplot(., aes(x = t, y = avg, color = lambda, group = lambda)) +
#       geom_line()
#   }

# data %>%
#   group_by(lambda) %>%
#   summarize(count = mean(count[-{1:10}]), .groups = "drop") %>% {
#     ggplot(., aes(x = lambda, y = count)) +
#       geom_line() +
#       geom_point()
#   }

# data %>%
#   filter(t > max(t) - 10) %>% {
#     ggplot(., aes(x = lambda, y = count, group = lambda, fill = lambda)) +
#       geom_boxplot()
#   }
