import argparse
import sys

import riva.client
import riva.client.audio_io

sys.stdout.reconfigure(line_buffering=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ASR Client')
    parser.add_argument('--server-uri', type=str,
                        default='localhost:50051', help='Server URI')
    parser.add_argument('--language-code', type=str,
                        default='en-US', help='Language code')
    parser.add_argument('--sample-rate-hz', type=int,
                        default=16000, help='Sample rate in Hz')
    parser.add_argument('--file-streaming-chunk', type=int,
                        default=1600, help='File streaming chunk size')
    parser.add_argument('--input-device', type=int,
                        default=0, help='Input device')
    parser.add_argument('--additional-info', type=str,
                        default='no', help='Additional info')
    args = parser.parse_args()

    auth = riva.client.Auth(uri=args.server_uri)

    asr_service = riva.client.ASRService(auth)

    config = riva.client.StreamingRecognitionConfig(
        config=riva.client.RecognitionConfig(
            encoding=riva.client.AudioEncoding.LINEAR_PCM,
            language_code=args.language_code,
            max_alternatives=1,
            profanity_filter=False,
            enable_automatic_punctuation=True,
            verbatim_transcripts=True,
            sample_rate_hertz=args.sample_rate_hz,
            audio_channel_count=1,
        ),
        interim_results=False,
    )

    with riva.client.audio_io.MicrophoneStream(
        args.sample_rate_hz,
        args.file_streaming_chunk,
        device=args.input_device,
    ) as audio_chunk_iterator:

        riva.client.print_streaming(
            responses=asr_service.streaming_response_generator(
                audio_chunks=audio_chunk_iterator,
                streaming_config=config,
            ),
            additional_info=args.additional_info,
        )
