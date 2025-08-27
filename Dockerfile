FROM flexradio/wfsdk-dev:alpine AS build

COPY . /tmp/build/
WORKDIR /tmp/build

RUN cmake -B cmake-build -S .

RUN cmake --build cmake-build --parallel=4

FROM alpine:latest AS example-waveform

LABEL com.flexradio.waveform.name=example
LABEL com.flexradio.waveform.version=1.0.0

COPY --from=build /tmp/build/cmake-build/waveform-example /app/waveform-example

ENTRYPOINT exec /app/waveform-example -h ${SSDR_RADIO_ADDRESS}
