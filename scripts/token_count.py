import nltk
nltk.download('punkt')
from nltk.tokenize import word_tokenize

def count_tokens(text):
    tokens = word_tokenize(text)
    return len(tokens)

# Example usage:
text = '''Make a summary of the method explained in the text given as INPUT, it's characteristics, pros and cons, why is was created and it's possible applications in Human Machine Interface.

INPUT: Abstract
We study the capabilities of speech processing
systems trained simply to predict large amounts of
transcripts of audio on the internet. When scaled
to 680,000 hours of multilingual and multitask
supervision, the resulting models generalize well
to standard benchmarks and are often competitive
with prior fully supervised results but in a zeroshot transfer setting without the need for any finetuning. When compared to humans, the models
approach their accuracy and robustness. We are
releasing models and inference code to serve as
a foundation for further work on robust speech
processing.
1. Introduction
Progress in speech recognition has been energized by the
development of unsupervised pre-training techniques exemplified by Wav2Vec 2.0 (Baevski et al., 2020). Since these
methods learn directly from raw audio without the need for
human labels, they can productively use large datasets of unlabeled speech and have been quickly scaled up to 1,000,000
hours of training data (Zhang et al., 2021), far more than the
1,000 or so hours typical of an academic supervised dataset.
When fine-tuned on standard benchmarks, this approach
has improved the state of the art, especially in a low-data
setting.
These pre-trained audio encoders learn high-quality representations of speech, but because they are purely unsupervised they lack an equivalently performant decoder mapping
those representations to usable outputs, necessitating a finetuning stage in order to actually perform a task such as
speech recognition1
. This unfortunately limits their usefulness and impact as fine-tuning can still be a complex
process requiring a skilled practitioner. There is an additional risk with requiring fine-tuning. Machine learning
*Equal contribution 1OpenAI, San Francisco, CA 94110, USA.
Correspondence to: Alec Radford <alec@openai.com>, Jong
Wook Kim <jongwook@openai.com>.
1Baevski et al. (2021) is an exciting exception - having developed a fully unsupervised speech recognition system
methods are exceedingly adept at finding patterns within a
training dataset which boost performance on held-out data
from the same dataset. However, some of these patterns are
brittle and spurious and don’t generalize to other datasets
and distributions. In a particularly disturbing example, Radford et al. (2021) documented a 9.2% increase in object
classification accuracy when fine-tuning a computer vision
model on the ImageNet dataset (Russakovsky et al., 2015)
without observing any improvement in average accuracy
when classifying the same objects on seven other natural
image datasets. A model that achieves “superhuman” performance when trained on a dataset can still make many
basic errors when evaluated on another, possibly precisely
because it is exploiting those dataset-specific quirks that
humans are oblivious to (Geirhos et al., 2020).
This suggests that while unsupervised pre-training has improved the quality of audio encoders dramatically, the lack
of an equivalently high-quality pre-trained decoder, combined with a recommended protocol of dataset-specific finetuning, is a crucial weakness which limits their usefulness
and robustness. The goal of a speech recognition system
should be to work reliably “out of the box” in a broad range
of environments without requiring supervised fine-tuning of
a decoder for every deployment distribution.
As demonstrated by Narayanan et al. (2018), Likhomanenko
et al. (2020), and Chan et al. (2021) speech recognition systems that are pre-trained in a supervised fashion across many
datasets/domains exhibit higher robustness and generalize
much more effectively to held-out datasets than models
trained on a single source. These works achieve this by
combining as many existing high-quality speech recognition datasets as possible. However, there is still only a
moderate amount of this data easily available. SpeechStew
(Chan et al., 2021) mixes together 7 pre-existing datasets
totalling 5,140 hours of supervision. While not insignificant, this is still tiny compared to the previously mentioned
1,000,000 hours of unlabeled speech data utilized in Zhang
et al. (2021).
Recognizing the limiting size of existing high-quality supervised datasets, recent efforts have created larger datasets for
speech recognition. By relaxing the requirement of goldstandard human-validated transcripts, Chen et al. (2021) and
Galvez et al. (2021) make use of sophisticated automated
Robust Speech Recognition via Large-Scale Weak Supervision 2
pipelines to scale weakly supervised speech recognition
to 10,000 and 30,000 hours of noisier training data. This
trade-off between quality and quantity is often the right
call. Although understudied so far for speech recognition,
recent work in computer vision has demonstrated that moving beyond gold-standard crowdsourced datasets such as
ImageNet (Russakovsky et al., 2015) to much larger but
weakly supervised datasets significantly improves the robustness and generalization of models (Mahajan et al., 2018;
Kolesnikov et al., 2020).
Yet these new datasets are only a few times larger than the
sum of existing high-quality datasets and still much smaller
than prior unsupervised work. In this work we close that
gap, scaling weakly supervised speech recognition the next
order of magnitude to 680,000 hours of labeled audio data.
We call our approach Whisper2
. We demonstrate models
trained at this scale transfer well to existing datasets zeroshot, removing the need for any dataset-specific fine-tuning
to achieve high-quality results.
In addition to scale, our work also focuses on broadening the scope of weakly supervised pre-training beyond
English-only speech recognition to be both multilingual and
multitask. Of those 680,000 hours of audio, 117,000 hours
cover 96 other languages. The dataset also includes 125,000
hours of X→en translation data. We find that for sufficiently
large models there is no drawback and even benefits to joint
multilingual and multitask training.
Our work suggests that simple scaling of weakly supervised
pre-training has been underappreciated so far for speech
recognition. We achieve these results without the need for
the self-supervision or self-training techniques that have
been a mainstay of recent large-scale speech recognition
work. To serve as a foundation for further research on robust
speech recognition, we release inference code and models at
the following URL: https://github.com/openai/
whisper.

5. Related Work
Scaling Speech Recognition A consistent theme across
speech recognition research has been documenting the benefits of scaling compute, models, and datasets. Early work applying deep learning to speech recognition found improved
performance with model depth and size and leveraged GPU
acceleration to make training these larger models tractable
(Mohamed et al., 2009). Further research demonstrated that
the benefit of deep learning approaches to speech recognition increased with dataset size, improving from being only
competitive with prior GMM-HMM systems when using
just 3 hours of TIMIT training data for phone recognition
to achieving a 30% word error rate reduction when trained
on the 2,000 hour Switchboard dataset (Seide et al., 2011).
Liao et al. (2013) is an early example of leveraging weakly
supervised learning to increase the size of a deep learning based speech recognition dataset by over 1,000 hours.
These trends continued with Deep Speech 2 (Amodei et al.,
2015) being a notable system developing high-throughput
distributed training across 16 GPUs and scaling to 12,000
hours of training data while demonstrating continuing improvements at that scale. By leveraging semi-supervised
pre-training, Narayanan et al. (2018) were able to grow
dataset size much further and study training on 162,000
hours of labeled audio. More recent work has explored
billion-parameter models (Zhang et al., 2020) and using up
to 1,000,000 hours of training data (Zhang et al., 2021).
Multitask Learning Multitask learning (Caruana, 1997)
has been studied for a long time. In speech recognition,
multi-lingual models have been explored for well over a
decade (Schultz & Kirchhoff, 2006). An inspirational and
foundational work in NLP exploring multi-task learning
with a single model is Collobert et al. (2011). Multitask
learning in the sequence-to-sequence framework (Sutskever
et al., 2014) using multiple encoders and decoders was investigated in Luong et al. (2015). The use of language codes
with a shared encoder/decoder architecture was first demonstrated for machine translation by Johnson et al. (2017),
removing the need for separate encoders and decoders. This
approach was simplified further into the “text-to-text” framework of McCann et al. (2018) and popularized by its success
with large transformer language models in the work of Radford et al. (2019) and Raffel et al. (2020). Toshniwal et al.
(2018) demonstrated jointly training a modern deep learning speech recognition system on several languages with a
single model, and Pratap et al. (2020a) scaled this line of
work significantly to 50 languages with a billion-parameter
model. MUTE (Wang et al., 2020c) and mSLAM (Bapna
et al., 2022) studied joint training over both text and speech
language tasks, demonstrating transfer between them.
Robustness The question of how effectively models transfer and how robust they are to distribution shift and other
types of perturbations has long been studied and is actively
being researched across many fields of machine learning.
Torralba & Efros (2011) highlighted the lack of generalization of machine learning models between datasets over a
decade ago. Many other works have shown and continually reiterated how despite high performance on IID test
sets, machine learning models can still make many mistakes
when evaluated in even slightly different settings (Lake et al.,
2017; Jia & Liang, 2017; Alcorn et al., 2019; Barbu et al.,
2019; Recht et al., 2019). More recently, Taori et al. (2020)
studied the robustness of image classification models, and
Miller et al. (2020) investigated this for question-answering
models. A key finding has been that multi-domain training increases robustness and generalization as discussed in
the Introduction. This finding has been replicated across
many fields in addition to speech recognition including NLP
(Hendrycks et al., 2020) and computer vision (Radford et al.,
2021).
6. Limitations and Future Work
From our experimental results, analyses, and ablations, we
have noted several limitations and areas for future work.
Robust Speech Recognition via Large-Scale Weak Supervision 14
Improved decoding strategies. As we have scaled Whisper, we have observed that larger models have made steady
and reliable progress on reducing perception-related errors
such as confusing similar-sounding words. Many remaining
errors, particularly in long-form transcription seem more
stubborn in nature and decidedly non-human/perceptual.
They are a combination of failure modes of seq2seq models, language models, and text-audio alignment and include
problems such as getting stuck in repeat loops, not transcribing the first or last few words of an audio segment, or
complete hallucination where the model will output a transcript entirely unrelated to the actual audio. Although the
decoding details discussed in Section 4.5 help significantly,
we suspect fine-tuning Whisper models on a high-quality
supervised dataset and/or using reinforcement learning to
more directly optimize for decoding performance could help
further reduce these errors.
Increase Training Data For Lower-Resource Languages
As Figure 3 shows, Whisper’s speech recognition performance is still quite poor on many languages. The same
analysis suggests a clear route for improvement since performance on a language is very well predicted by the amount
of training data for the language. Since our pre-training
dataset is currently very English-heavy due to biases of
our data collection pipeline, which sourced primarily from
English-centric parts of the internet, most languages have
less than 1000 hours of training data. A targeted effort at increasing the amount of data for these rarer languages could
result in a large improvement to average speech recognition
performance even with only a small increase in our overall
training dataset size.
Studying fine-tuning In this work, we have focused on
the robustness properties of speech processing systems and
as a result only studied the zero-shot transfer performance
of Whisper. While this is a crucial setting to study due to it
being representative of general reliability, for many domains
where high-quality supervised speech data does exist, it is
likely that results can be improved further by fine-tuning.
An additional benefit of studying fine-tuning is that it allows
for direct comparisons with prior work since it is a much
more common evaluation setting.
Studying the impact of Language Models on Robustness
As argued in the introduction, we suspect that Whisper’s
robustness is partially due to its strong decoder, which is an
audio conditional language model. It’s currently unclear to
what degree the benefits of Whisper stem from training its
encoder, decoder, or both. This could be studied by either
ablating various design components of Whisper, such as
training a decoder-less CTC model, or by studying how the
performance of existing speech recognition encoders such
as wav2vec 2.0 change when used together with a language
model.
Adding Auxiliary Training Objectives Whisper departs
noticeably from most recent state-of-the-art speech recognition systems due to the lack of unsupervised pre-training
or self-teaching methods. While we have not found them
necessary to achieve good performance, it is possible that
the results could be further improved by incorporating this.
7. Conclusion
Whisper suggests that scaling weakly supervised pretraining has been underappreciated so far in speech recognition research. We achieve our results without the need for
the self-supervision and self-training techniques that have
been a mainstay of recent large-scale speech recognition
work and demonstrate how simply training on a large and
diverse supervised dataset and focusing on zero-shot transfer can significantly improve the robustness of a speech
recognition system.
'''

token_count = count_tokens(text)
print(f"The number of tokens in the text is: {token_count}")