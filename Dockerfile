FROM ruby:3

WORKDIR /usr/src/app

COPY Gemfile Gemfile.lock ./
RUN bundle install
COPY . .
EXPOSE 3000
CMD ["bundle", "exec", "nanoc", "live", "-o", "0.0.0.0"]
